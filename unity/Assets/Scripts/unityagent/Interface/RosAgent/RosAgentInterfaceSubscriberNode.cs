using System;
using System.Collections.Generic;

using Unity.Robotics.ROSTCPConnector.MessageGeneration;

using UnityAgent.SystemMessage;

namespace UnityAgent.Interface.RosAgent
{
public class RosAgentInterfaceSubscriberNode : RosAgentInterfaceNode
{
    private static byte nodeType = 1;
    private List<Action<Message, ushort>> subscribers = new();

    public RosAgentInterfaceSubscriberNode(Action<Message, ushort> subscriber,
                                           string tosName,
                                           string dataExchangeType,
                                           uint rosAgentBufSize,
                                           bool dropNew)
    : base(nodeType, tosName, dataExchangeType, rosAgentBufSize, dropNew)
    {
        subscribers.Add(subscriber);
    }

    public bool HasSubscriber(Action<Message, ushort> callback)
    {
        return subscribers.Contains(callback);
    }

    public void AddSubscriber(Action<Message, ushort> callback)
    {
        if (!HasSubscriber(callback))
            subscribers.Add(callback);
    }

    public void RemoveSubscriber(Action<Message, ushort> callback)
    {
        subscribers.Remove(callback);
    }

    public override void ProcessIsm(IngressSystemMessage ism)
    {
        Message message = DeserializeMessageData(ism.Data);
        ApplySubscribers(message, ism.SenderAgentId);
    }

    private void ApplySubscribers(Message message, ushort senderAgentId)
    {
        subscribers.ForEach(subscriber => subscriber(message, senderAgentId));
    }
}
}