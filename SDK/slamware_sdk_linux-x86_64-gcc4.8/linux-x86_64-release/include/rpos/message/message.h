/*
* message.h
* Envolop of messages
*
* Was drv_slamsdp/rp/slamware/message/message.h
*
* Created by Tony Huang at 2016-11-15
* Copyright 2016 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include <cstdint>

namespace rpos { namespace message {

	typedef std::uint64_t message_timestamp_t;

	template < typename TPayload >
	struct Message
	{
		Message()
			: timestamp(0)
		{
		}

		explicit Message(const TPayload& that)
			: timestamp(0)
			, payload(that)
		{
		}

		message_timestamp_t timestamp;
		TPayload payload;

		inline TPayload& operator*()
		{
			return payload;
		}

		inline TPayload* operator->()
		{
			return &payload;
		}

		inline const TPayload& operator*() const
		{
			return payload;
		}

		inline const TPayload* operator->() const
		{
			return &payload;
		}

		Message<TPayload>& operator=(const TPayload& that)
		{
			payload = that;
			return *this;
		}
	};

	template < typename TPayload >
	static inline bool updateIfNewer(Message<TPayload>& target, const Message<TPayload>& ref)
	{
		if (ref.timestamp > target.timestamp)
		{
			target = ref;
			return true;
		}

		return false;
	}

	template < typename TPayload>
	static inline bool updateIfDifferent(Message<TPayload>& target, const Message<TPayload>& ref)
	{
		if (ref.timestamp != target.timestamp)
		{
			target = ref;
			return true;
		}

		return false;
	}

} }
