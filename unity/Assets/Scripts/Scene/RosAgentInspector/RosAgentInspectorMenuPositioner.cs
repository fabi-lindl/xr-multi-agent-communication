using System.Collections.Generic;

using UnityEngine;

namespace Scene.RosAgentInspectorMenu
{
public static class RosAgentInspectorMenuPositioner
{
    private const float distanceFromHeadset = 1; // Meters.
    private static Dictionary<int, float> yAnglesToDegrees = new()
    {
        { 0,   0 },
        { 1,  20 },
        { 2, -20 },
        { 3,  40 },
        { 4, -40 },
        { 5,  60 },
        { 6, -60 },
    };

    public static Vector3 GetPosition(int positionId)
    {
        float yAngleDeg = yAnglesToDegrees[positionId];
        float yAngleRad = yAngleDeg * Mathf.Deg2Rad;
        float x = distanceFromHeadset * Mathf.Sin(yAngleRad);
        float y = 1;
        float z = distanceFromHeadset * Mathf.Cos(yAngleRad);
        Vector3 position = new Vector3(x, y, z);
        return position;
    }

    public static Quaternion GetRotation(int positionId)
    {
        float yAngle = yAnglesToDegrees[positionId];
        Vector3 eulerAngles = new Vector3(0, yAngle, 0);
        Quaternion rotation = new();
        rotation.eulerAngles = eulerAngles;
        return rotation;
    }
}
}