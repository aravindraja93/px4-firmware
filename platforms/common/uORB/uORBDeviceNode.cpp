/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <sys/shm.h>
#include <sys/mman.h>

#include "uORBDeviceNode.hpp"

#include "uORBUtils.hpp"
#include "uORBManager.hpp"

#include "SubscriptionCallback.hpp"

#ifdef ORB_COMMUNICATOR
#include "uORBCommunicator.hpp"
#endif /* ORB_COMMUNICATOR */

#if defined(__PX4_NUTTX)
#include <nuttx/arch.h>
#include <nuttx/mm/mm.h>
#endif

#include <px4_platform_common/sem.hpp>

// round up to nearest power of two
// Such as 0 => 1, 1 => 1, 2 => 2 ,3 => 4, 10 => 16, 60 => 64, 65...255 => 128
// Note: When the input value > 128, the output is always 128
static inline uint8_t round_pow_of_two_8(uint8_t n)
{
	if (n == 0) {
		return 1;
	}

	// Avoid is already a power of 2
	uint8_t value = n - 1;

	// Fill 1
	value |= value >> 1U;
	value |= value >> 2U;
	value |= value >> 4U;

	// Unable to round-up, take the value of round-down
	if (value == UINT8_MAX) {
		value >>= 1U;
	}

	return value + 1;
}

orb_advert_t uORB::DeviceNode::nodeOpen(const struct orb_metadata *meta, const uint8_t instance, bool create)
{

	/*
	 * Generate the path to the node and try to open it.
	 */

	char nodepath[orb_maxpath];
	int inst = instance;
	int ret = uORB::Utils::node_mkpath(nodepath, meta, &inst);
	bool created = false;

	if (ret != OK) {
		return ORB_ADVERT_INVALID;
	}

	// First, try to open an existing node

	int shm_fd = shm_open(nodepath, O_RDWR, 0666);

	if (shm_fd < 0 && create) {
		// Create a new one if requested. This fails if it alread exists
		// Using O_EXCL only to detect any mutual exclusion problem in opening

		shm_fd = shm_open(nodepath, O_CREAT | O_RDWR | O_EXCL, 0666);

		if (shm_fd >= 0) {

			// If the creation succeeded, set the size of the shm region
			if (ftruncate(shm_fd, sizeof(uORB::DeviceNode)) != 0) {
				::close(shm_fd);
				shm_fd = -1;

			} else {
				created = true;
			}

		} else {
			// We should never end up in here
			PX4_ERR("Node open failed\n");
		}

	}

	if (shm_fd < 0) {
		// We were not able to create a new node or open an exiting one
		return ORB_ADVERT_INVALID;
	}

	uORB::DeviceNode *node = nullptr;

	// mmap the shared memory region
	void *ptr = mmap(0, sizeof(uORB::DeviceNode), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);

	if (ptr != MAP_FAILED) {
		if (created) {
			// construct the new node in the region
			node = new (ptr) uORB::DeviceNode(meta, instance, nodepath, 0);

		} else {
			// take the existing node into use
			node = static_cast<uORB::DeviceNode *>(ptr);
		}
	}

	// No need to keep the fd any more, close it

	::close(shm_fd);

	return {node, nullptr, 0};
}

int uORB::DeviceNode::nodeClose(orb_advert_t &handle)
{
	if (!orb_advert_valid(handle)) {
		return PX4_ERROR;
	}

	// If the data has been mapped, unmap that
	if (node_data(handle) != nullptr) {
		munmap(node_data(handle), node(handle)->_queue_size * node(handle)->get_meta()->o_size);
	}

	// Unmap the node
	munmap(node(handle), sizeof(uORB::DeviceNode));

	handle = ORB_ADVERT_INVALID;

	return PX4_OK;
}

orb_advert_t uORB::DeviceNode::advertise(const struct orb_metadata *meta, int *instance, unsigned queue_size)
{
	orb_advert_t handle;
	int ret = PX4_ERROR;

	/* try for topic groups */
	const unsigned max_group_tries = (instance != nullptr) ? ORB_MULTI_MAX_INSTANCES : 1;
	unsigned group_tries = 0;

	if (instance) {
		group_tries = *instance;
	}

	while (ret != PX4_OK && (group_tries < max_group_tries)) {
		if (instance != nullptr) {
			*instance = group_tries;
		}

		/* Open the node, if it exists */
		handle = nodeOpen(meta, group_tries, false);

		if (!orb_advert_valid(handle)) {
			/* Doesn't exist, create */
			handle = nodeOpen(meta, group_tries, true);
			ret = PX4_OK;

		} else {
			/* Already exists
			 * Claim it if not already advertised or is a single instance advertiser
			 */
			if (!is_advertised(handle) || !instance) {
				ret = PX4_OK;

			} else {
				/* Call to nodeClose also resets the handle */
				nodeClose(handle);
			}
		}

		group_tries++;

	}

	if (orb_advert_valid(handle)) {
		node(handle)->_advertiser_count++;
		node(handle)->update_queue_size(queue_size);
	}

	return handle;
}

int uORB::DeviceNode::unadvertise(orb_advert_t &handle)
{
	_advertiser_count--;

	// TODO! Handle all subscriber callbacks?

	nodeClose(handle);

	return PX4_OK;
}

uORB::DeviceNode::DeviceNode(const struct orb_metadata *meta, const uint8_t instance, const char *path,
			     uint8_t queue_size) :
	_orb_id(static_cast<ORB_ID>(meta->o_id)),
	_instance(instance),
	_queue_size(round_pow_of_two_8(queue_size))
{
	strncpy(_devname, path, sizeof(_devname));

	int ret = px4_sem_init(&_lock, 0, 1);

	if (ret != 0) {
		PX4_DEBUG("SEM INIT FAIL: ret %d", ret);
	}
}

uORB::DeviceNode::~DeviceNode()
{
	shm_unlink(get_devname());
	px4_sem_destroy(&_lock);
}

void uORB::DeviceNode::remap_data(orb_advert_t &handle, bool advertiser)
{
	size_t old_size = handle.queue_size * get_meta()->o_size;
	size_t new_size = _queue_size * get_meta()->o_size;

	if (old_size == new_size) {
		return;
	}

	if (node_data(handle) != nullptr) {
		munmap(node_data(handle), old_size);
	}

	handle.node_data = nullptr;

	// Set the new queue size
	handle.queue_size = _queue_size;

	// Open the data, the data shm name is the same as device node's except for leading '_'
	int oflag = advertiser ? O_RDWR | O_CREAT : O_RDONLY;
	int shm_fd = shm_open(get_devname() + 1, oflag, 0666);

	// and mmap it
	if (shm_fd >= 0) {

		// For the advertiser, set the new data size
		if (advertiser) {
			if (ftruncate(shm_fd, new_size) != 0) {
				::close(shm_fd);
				return;
			}
		}

		handle.node_data = mmap(0, new_size, advertiser ? PROT_WRITE : PROT_READ, MAP_SHARED, shm_fd, 0);

		if (handle.node_data == MAP_FAILED) {
			handle.node_data = nullptr;
		}

		::close(shm_fd);

	}
}

/**
	 * Copies data and the corresponding generation
	 * from a node to the buffer provided.
	 *
	 * @param dst
	 *   The buffer into which the data is copied.
	 * @param generation
	 *   The generation that was copied.
	 * @return bool
	 *   Returns true if the data was copied.
	 */
bool uORB::DeviceNode::copy(void *dst, orb_advert_t &handle, unsigned &generation)
{
	if (dst == nullptr) {
		return false;
	}

	lock();
	remap_data(handle, false);

	if (node_data(handle) == nullptr) {
		unlock();
		return false;
	}

	if (_queue_size == 1) {
		memcpy(dst, node_data(handle), get_meta()->o_size);
		generation = _generation.load();

	} else {
		const unsigned current_generation = _generation.load();

		if (current_generation == generation) {
			/* The subscriber already read the latest message, but nothing new was published yet.
			 * Return the previous message
			 */
			--generation;
		}

		// Compatible with normal and overflow conditions
		if (!is_in_range(current_generation - _queue_size, generation, current_generation - 1)) {
			// Reader is too far behind: some messages are lost
			generation = current_generation - _queue_size;
		}

		memcpy(dst, ((uint8_t *)node_data(handle)) + (get_meta()->o_size * (generation % _queue_size)), get_meta()->o_size);

		++generation;
	}

	unlock();

	return true;

}

ssize_t
uORB::DeviceNode::write(const char *buffer, size_t buflen, orb_advert_t &handle)
{
	/*
	 * Writes are legal from interrupt context as long as the
	 * object has already been initialised from thread context.
	 */

	/* If write size does not match, that is an error */
	if (get_orb_meta(_orb_id)->o_size != buflen) {
		return -EIO;
	}

	/* Perform an atomic copy. */

	lock();
	remap_data(handle, true);

	if (node_data(handle) == nullptr) {
		return -ENOMEM;
	}

	/* wrap-around happens after ~49 days, assuming a publisher rate of 1 kHz */
	unsigned generation = _generation.fetch_add(1);

	memcpy(((uint8_t *)node_data(handle)) + get_meta()->o_size * (generation % _queue_size), buffer, get_meta()->o_size);

	// callbacks
	for (auto item : _callbacks) {
		item->call();
	}

	/* Mark at least one data has been published */
	_data_valid = true;

	unlock();

	/* notify any poll waiters */
	//TODO	orb_poll_notify(POLLIN);

	return get_meta()->o_size;
}

ssize_t
uORB::DeviceNode::publish(const orb_metadata *meta, orb_advert_t &handle, const void *data)
{
	uORB::DeviceNode *devnode = node(handle);
	int ret;

	/* check if the device handle is initialized and data is valid */
	if ((devnode == nullptr) || (meta == nullptr) || (data == nullptr)) {
		errno = EFAULT;
		return PX4_ERROR;
	}

	/* check if the orb meta data matches the publication */
	if (devnode->get_meta()->o_id != meta->o_id) {
		errno = EINVAL;
		return PX4_ERROR;
	}

	/* call the devnode write method */
	ret = devnode->write((const char *)data, meta->o_size, handle);

	if (ret < 0) {
		errno = -ret;
		return PX4_ERROR;
	}

#ifdef ORB_COMMUNICATOR
	/*
	 * if the write is successful, send the data over the Multi-ORB link
	 */
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr) {
		if (ch->send_message(meta->o_name, meta->o_size, (uint8_t *)data) != 0) {
			PX4_ERR("Error Sending [%s] topic data over comm_channel", meta->o_name);
			return PX4_ERROR;
		}
	}

#endif /* ORB_COMMUNICATOR */

	return PX4_OK;
}

#ifdef ORB_COMMUNICATOR
int16_t uORB::DeviceNode::topic_advertised(const orb_metadata *meta)
{
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr && meta != nullptr) {
		return ch->topic_advertised(meta->o_name);
	}

	return -1;
}

/*
//TODO: Check if we need this since we only unadvertise when things all shutdown and it doesn't actually remove the device
int16_t uORB::DeviceNode::topic_unadvertised(const orb_metadata *meta)
{
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();
	if (ch != nullptr && meta != nullptr) {
		return ch->topic_unadvertised(meta->o_name);
	}
	return -1;
}
*/
#endif /* ORB_COMMUNICATOR */

bool
uORB::DeviceNode::print_statistics(int max_topic_length)
{
	if (_advertiser_count == 0) {
		return false;
	}

	lock();

	const uint8_t instance = get_instance();
	const int8_t sub_count = subscriber_count();
	const uint8_t queue_size = get_queue_size();

	unlock();

	PX4_INFO_RAW("%-*s %2i %4i %2i %4i %s\n", max_topic_length, get_meta()->o_name, (int)instance, (int)sub_count,
		     queue_size, get_meta()->o_size, get_devname());

	return true;
}

bool uORB::DeviceNode::add_subscriber(const orb_advert_t &handle, unsigned *initial_generation)
{
	bool ret;

	if (orb_advert_valid(handle)) {
		node(handle)->add_internal_subscriber(initial_generation);
		ret = true;

	} else {
		*initial_generation = 0;
		ret = false;
	}

	return ret;
}

void uORB::DeviceNode::add_internal_subscriber(unsigned *initial_generation)
{
	*initial_generation = get_initial_generation();

	lock();
	_subscriber_count++;

#ifdef ORB_COMMUNICATOR
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr && _subscriber_count > 0) {
		unlock(); //make sure we cannot deadlock if add_subscription calls back into DeviceNode
		ch->add_subscription(get_meta()->o_name, 1);

	} else
#endif /* ORB_COMMUNICATOR */

	{
		unlock();
	}
}

void uORB::DeviceNode::remove_subscriber(orb_advert_t &handle)
{
	if (orb_advert_valid(handle)) {
		node(handle)->remove_internal_subscriber(handle);
	}
}

void uORB::DeviceNode::remove_internal_subscriber(const orb_advert_t &handle)
{
	lock();
	_subscriber_count--;

#ifdef ORB_COMMUNICATOR
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr && _subscriber_count == 0) {
		unlock(); //make sure we cannot deadlock if remove_subscription calls back into DeviceNode
		ch->remove_subscription(get_meta()->o_name);

	} else
#endif /* ORB_COMMUNICATOR */
	{
		unlock();
	}
}

#ifdef ORB_COMMUNICATOR
int16_t uORB::DeviceNode::process_add_subscription(int32_t rateInHz)
{
	// if there is already data in the node, send this out to
	// the remote entity.
	// send the data to the remote entity.
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr) {
		ch->send_message(get_meta()->o_name, get_meta()->o_size, _data);
	}

	return PX4_OK;
}

int16_t uORB::DeviceNode::process_remove_subscription()
{
	return PX4_OK;
}

int16_t uORB::DeviceNode::process_received_message(int32_t length, uint8_t *data)
{
	int16_t ret = -1;

	if (length != (int32_t)(get_meta()->o_size)) {
		PX4_ERR("Received '%s' with DataLength[%d] != ExpectedLen[%d]", get_meta()->o_name, (int)length,
			(int)get_meta()->o_size);
		return PX4_ERROR;
	}

	/* call the devnode write method */
	ret = write((const char *)data, get_meta()->o_size);

	if (ret < 0) {
		return PX4_ERROR;
	}

	if (ret != (int)get_meta()->o_size) {
		errno = EIO;
		return PX4_ERROR;
	}

	return PX4_OK;
}
#endif /* ORB_COMMUNICATOR */

int uORB::DeviceNode::update_queue_size(unsigned int queue_size)
{
	if (_queue_size == queue_size) {
		return PX4_OK;
	}

	//queue size is limited to 255 for the single reason that we use uint8 to store it
	if (_queue_size > queue_size || queue_size > 255) {
		return PX4_ERROR;
	}

	_queue_size = round_pow_of_two_8(queue_size);
	return PX4_OK;
}

unsigned uORB::DeviceNode::get_initial_generation()
{
	lock();

	// If there any previous publications allow the subscriber to read them
	unsigned generation = _generation.load() - (_data_valid ? 1 : 0);

	unlock();

	return generation;
}

bool
uORB::DeviceNode::register_callback(uORB::SubscriptionCallback *callback_sub)
{
	if (callback_sub != nullptr) {
		lock();

		// prevent duplicate registrations
		for (auto existing_callbacks : _callbacks) {
			if (callback_sub == existing_callbacks) {
				lock();
				return true;
			}
		}

		_callbacks.add(callback_sub);
		unlock();
		return true;
	}

	return false;
}

void
uORB::DeviceNode::unregister_callback(uORB::SubscriptionCallback *callback_sub)
{
	lock();
	_callbacks.remove(callback_sub);
	unlock();
}
