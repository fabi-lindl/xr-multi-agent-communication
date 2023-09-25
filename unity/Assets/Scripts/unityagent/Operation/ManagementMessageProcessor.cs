using System;
using UnityAgent.Management.Manager;
using UnityAgent.SystemMessage;

namespace UnityAgent.Operation
{
public static class ManagementMessageProcessor
{
    private static readonly string messageManagerNamespace = "UnityAgent." +
                                                      "Management.Manager.";

    public static void ProcessIsm(IngressSystemMessage ism)
    {
        if (ism == null) return;
        ManagementMessageManager manager = GetMessageManager(ism);
        manager?.ProcessIsm(ism);
    }

    private static ManagementMessageManager GetMessageManager(IngressSystemMessage ism)
    {
        string className = ism.MessageManager;
        string classNamespacePath = messageManagerNamespace + className;
        Type t = Type.GetType(classNamespacePath);
        if (!MessageManagerExists(t)) return null;
        ManagementMessageManager manager = (ManagementMessageManager)Activator
                                           .CreateInstance(t);
        return manager;
    }

    private static bool MessageManagerExists(Type t)
    {
        return t != null;
    }
}
}