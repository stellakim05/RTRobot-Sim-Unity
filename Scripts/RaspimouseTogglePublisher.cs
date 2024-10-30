using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

public class RaspimouseTogglePublisher : MonoBehaviour
{
    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<BoolMsg>("/unity_movement_feedback");
        ros.RegisterPublisher<TwistMsg>("/raspimouse1/cmd_vel");
        ros.RegisterPublisher<TwistMsg>("/raspimouse2/cmd_vel");
    }

    // This method is mainly for debugging purposes
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            ToggleMovement();
        }
    }

    void ToggleMovement()
    {
        Debug.Log("Unity: Toggling movement");
        TwistMsg msg = new TwistMsg();
        msg.linear.z = 0.2; // Set to 0 to stop, 0.2 to move
        ros.Publish("/raspimouse1/cmd_vel", msg);
        ros.Publish("/raspimouse2/cmd_vel", msg);
    }
}