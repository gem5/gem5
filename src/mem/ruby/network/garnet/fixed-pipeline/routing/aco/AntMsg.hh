#ifndef __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_ROUTING_ACO_ANT_MSG_HH__
#define __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_ROUTING_ACO_ANT_MSG_HH__

#include <iostream>
#include <vector>

#include "mem/ruby/slicc_interface/RubySlicc_Util.hh"
#include "mem/ruby/common/TypeDefines.hh"
#include "mem/protocol/CoherenceRequestType.hh"
#include "mem/protocol/MachineID.hh"
#include "mem/protocol/NetDest.hh"
#include "mem/protocol/DataBlock.hh"
#include "mem/protocol/MessageSizeType.hh"
#include "mem/protocol/Message.hh"

// Ant message.
class AntMsg: public Message
{
    public:
        // Create an ant message.
        AntMsg(const Tick curTime, bool forward, int source, int destination);

        // Set a boolean value indicating whether it is a forward ant message or a backward ant message.
        void setForward(bool forward);

        // Get a boolean value indicating whether it is a forward ant message or a backward ant message.
        bool isForward();

        // Set the source router.
        void setSourceRouter(int sourceRouter);

        // Get the source router.
        int getSourceRouter();

        // Set the destination router.
        void setDestinationRouter(int destinationRouter);

        // Get the destination router.
        int getDestinationRouter();

        // Get the memory.
        std::vector<int>& getMemory();

        MsgPtr
        clone() const
        {
             return std::shared_ptr<Message>(new AntMsg(*this));
        }

        void print(std::ostream& out) const;

        const MessageSizeType& getMessageSize() const
        {
            return m_MessageSize;
        }

        MessageSizeType& getMessageSize()
        {
            return m_MessageSize;
        }

        /** \brief Const accessor method for Destination field.
         *  \return Destination field
         */
        const NetDest&
        getDestination() const
        {
            return m_Destination;
        }

        /** \brief Const accessor method for Destination field.
         *  \return Destination field
         */
        NetDest&
        getDestination()
        {
            return m_Destination;
        }

        bool functionalWrite(Packet* param_pkt);

        bool functionalRead(Packet* param_pkt);

      //private:
        Addr m_addr;

        /** Multicast destination mask */
        NetDest m_Destination;

        /** size category of the message */
        MessageSizeType m_MessageSize;

    private:
        bool m_forward;
        int m_sourceRouter;
        int m_destinationRouter;
        std::vector<int> m_memory;
};


#endif // __MEM_RUBY_NETWORK_GARNET_FIXED_PIPELINE_ROUTING_ACO_ANT_MSG_HH__