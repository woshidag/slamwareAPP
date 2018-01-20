/*
* message_write_stream.h
* Message write stream
*
* Created by Tony Huang (tony@slamtec.com) at 2017-3-14
* Copyright 2017 (c) Shanghai Slamtec Co., Ltd.
*/

#pragma once

#include "i_message_write_stream.h"

namespace rpos { namespace system { namespace diagnosis {

    namespace detail {
        RPOS_CORE_API std::string getTypeName(const std::type_index& typeIndex);
    }

    class RPOS_CORE_API MessageWriteStream : public IMessageWriteStream {
    public:
        MessageWriteStream(boost::shared_ptr<io::IStream> metadataStream, boost::shared_ptr<io::IStream> messageStream);
        MessageWriteStream(boost::shared_ptr<io::IStream> underlyingStream);
        ~MessageWriteStream();

    public:
        boost::shared_ptr<io::IStream> metadataStream();
        boost::shared_ptr<io::IStream> messageStream();

    protected:
        virtual void writeMessage(const std::string& topic, const std::type_index& typeIndex, const io::MemoryWriteStream& body, MessageWriteFlags flags);

    private:
        std::uint8_t ensureTypeStream(const std::string& topic, const std::type_index& typeIndex);

    private:
        boost::mutex lock_;
        boost::shared_ptr<io::IStream> metadataStream_;
        boost::shared_ptr<io::IStream> messageStream_;
        std::uint8_t nextStreamId_;
        boost::unordered_map<std::string, std::uint8_t> typeStreamMap_;
    };

} } }
