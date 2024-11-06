using UnityEngine;
using System.Collections.Generic;
using System;
using System.IO;

public class ExperimentDataCollector : MonoBehaviour
{
    [Header("Objects References")]
    public GameObject box;
    public GameObject robot1;
    public GameObject robot2;
    public HapticPlugin hapticDevice;

    [Header("Recording Settings")]
    public float samplingRate = 0.02f; // 50Hz
    public bool autoStartRecording = false;

    [Header("Experimental Parameters")]
    public float targetRotation = 0f;  // Target Y rotation
    public float rotationThreshold = 5f;  // Degrees
    public float contactForceThreshold = 0.01f;  // Minimum force to count as contact

    [Header("Debug")]
    public bool showTimeDebug = true;
    
    // Timing variables
    private float experimentStartTime;
    private float currentTaskTime;
    private bool isRecording = false;
    private float lastSampleTime;
    
    // Session info
    private string sessionID;
    private string dataPath;
    private StreamWriter writer;

    // Performance metrics
    private float totalRotationError = 0f;
    private int rotationErrorSamples = 0;
    private float maxRotationDeviation = 0f;
    private int stabilityViolations = 0;
    private float cumulativeContactTime = 0f;
    private float lastContactTime = 0f;

    // Phase tracking
    private string currentPhase = "initialization";
    private bool inContactPhase = false;
    private float contactStartTime = 0f;

    void Start()
    {
        InitializeSession();
        if (autoStartRecording)
        {
            StartRecording();
        }
    }

    void InitializeSession()
    {
        sessionID = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        dataPath = Path.Combine(Application.dataPath, "ExperimentData");
        Directory.CreateDirectory(dataPath);
        currentTaskTime = 0f;
    }

    public void StartRecording()
    {
        if (isRecording)
        {
            Debug.LogWarning("Already recording!");
            return;
        }

        experimentStartTime = Time.time;
        lastSampleTime = Time.time;
        isRecording = true;
        currentTaskTime = 0f;
        
        string filename = Path.Combine(dataPath, $"experiment_session_{sessionID}.csv");
        writer = new StreamWriter(filename, false);
        WriteHeader();
        
        Debug.Log($"Started recording session: {sessionID} at time: {experimentStartTime}");
    }

    void WriteHeader()
    {
        writer.WriteLine(
            "TaskTime,BoxRotation,RotationError,BoxPosX,BoxPosY,BoxPosZ," +
            "BoxAngVelX,BoxAngVelY,BoxAngVelZ," +
            "Robot1PosX,Robot1PosY,Robot1PosZ,Robot2PosX,Robot2PosY,Robot2PosZ," +
            "Robot1Speed,Robot2Speed,RobotDistanceDiff," +
            "HapticPosX,HapticPosY,HapticPosZ," +
            "HapticForceX,HapticForceY,HapticForceZ,ForceMagnitude," +
            "IsInContact,ContactCount,ContactDuration,Phase," +
            "CumulativeError,StabilityMetric"
        );
    }

    void Update()
    {
        if (!isRecording) return;

        // Update task time
        currentTaskTime = Time.time - experimentStartTime;

        if (showTimeDebug)
        {
            Debug.Log($"Task Time: {currentTaskTime:F2} seconds");
        }

        // Record data at specified sampling rate
        if (Time.time - lastSampleTime >= samplingRate)
        {
            RecordDataPoint();
            lastSampleTime = Time.time;
        }
    }

    void RecordDataPoint()
    {
        if (writer == null) return;

        // Box metrics
        float boxRotation = box.transform.eulerAngles.y;
        if (boxRotation > 180f) boxRotation -= 360f;
        // Calculate error based on target angle (0 degrees)
        float rotationError = Mathf.Abs(boxRotation);
        Vector3 boxPosition = box.transform.position;
        Vector3 boxAngularVelocity = box.GetComponent<Rigidbody>().angularVelocity;

        // Update performance metrics
        totalRotationError += rotationError;
        rotationErrorSamples++;
        maxRotationDeviation = Mathf.Max(maxRotationDeviation, rotationError);
        if (rotationError > rotationThreshold)
            stabilityViolations++;

        // Get haptic collider contact information
        bool isInContact = false;
        int contactCount = 0;
        float contactDuration = 0f;

        // Check for any active contacts with significant force
        if (hapticDevice.MagForce > 0.01f)  // Adjust threshold as needed
        {
            Collider[] colliders = Physics.OverlapSphere(hapticDevice.CollisionMesh.transform.position, 0.1f);  // Adjust radius as needed
            foreach (Collider collider in colliders)
            {
                if (collider.gameObject == box || 
                    collider.gameObject.transform.IsChildOf(robot1.transform) ||
                    collider.gameObject.transform.IsChildOf(robot2.transform))
                {
                    isInContact = true;
                    contactCount++;
                    Debug.Log($"Contact detected with: {collider.gameObject.name}");
                }
            }
        }

        // Alternative contact detection using force threshold (when at resting or touching floor)
        if (!isInContact && hapticDevice.MagForce > 0.01f)  // Backup check using force
        {
            isInContact = false;
            contactCount = 0;
            Debug.Log($"Contact detected via force: {hapticDevice.MagForce}");
        }

        // Update contact duration and phase
        if (isInContact)
        {
            if (!inContactPhase)
            {
                contactStartTime = currentTaskTime;
                inContactPhase = true;
            }
            contactDuration = currentTaskTime - contactStartTime;
            cumulativeContactTime += Time.deltaTime;
        }
        else
        {
            inContactPhase = false;
        }

        // Robot metrics - using ArticulationBody for correct velocities
        Vector3 robot1Position = robot1.transform.position;
        Vector3 robot2Position = robot2.transform.position;
        float robot1Speed = robot1.GetComponent<ArticulationBody>().velocity.magnitude;
        float robot2Speed = robot2.GetComponent<ArticulationBody>().velocity.magnitude;
        float robotDistance = Vector3.Distance(robot1Position, robot2Position);

        // Haptic metrics
        Vector3 hapticPosition = hapticDevice.transform.position;
        Vector3 hapticForce = hapticDevice.CurrentForce;
        float forceMagnitude = hapticDevice.MagForce;

        // Update experiment phase
        UpdateExperimentPhase(boxRotation, rotationError, isInContact);

        // Calculate stability metric
        float stabilityMetric = CalculateStabilityMetric(rotationError, forceMagnitude);

        // Write data point with new metrics
        writer.WriteLine(string.Format(
            "{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15},{16},{17}," +
            "{18},{19},{20},{21},{22},{23},{24},{25},{26},{27},{28},{29},{30}",
            currentTaskTime,
            boxRotation, rotationError,
            boxPosition.x, boxPosition.y, boxPosition.z,
            boxAngularVelocity.x, boxAngularVelocity.y, boxAngularVelocity.z,
            robot1Position.x, robot1Position.y, robot1Position.z,
            robot2Position.x, robot2Position.y, robot2Position.z,
            robot1Speed, robot2Speed, robotDistance,
            hapticPosition.x, hapticPosition.y, hapticPosition.z,
            hapticForce.x, hapticForce.y, hapticForce.z,
            forceMagnitude,
            isInContact ? 1 : 0,
            contactCount,
            contactDuration,
            currentPhase,
            totalRotationError / (rotationErrorSamples > 0 ? rotationErrorSamples : 1),
            stabilityMetric
        ));
    }

    private void UpdateExperimentPhase(float boxRotation, float rotationError, bool isInContact)
        {
            if (!isInContact && !inContactPhase)
                currentPhase = "initialization";
            else if (isInContact && rotationError < rotationThreshold)
                currentPhase = "stable_contact";
            else if (isInContact && rotationError >= rotationThreshold)
                currentPhase = "correction";
            else if (!isInContact && inContactPhase)
                currentPhase = "release";
        }

    private float CalculateStabilityMetric(float rotationError, float forceMagnitude)
        {
            // Normalize values (0 to 1 range)
            float normalizedError = rotationError / 180f;
            float normalizedForce = forceMagnitude / 5f; // Assuming 5N as max force

            // Combine metrics (lower is better)
            return (normalizedError + normalizedForce) / 2f;
        }

    public void StopRecording()
    {
        if (!isRecording) return;

        isRecording = false;
        float finalTaskTime = Time.time - experimentStartTime;

        if (writer != null)
        {
            // Enhanced summary statistics
            writer.WriteLine("\nSession Summary");
            writer.WriteLine($"Total Task Time: {finalTaskTime:F2} seconds");
            writer.WriteLine($"Average Rotation Error: {(rotationErrorSamples > 0 ? totalRotationError / rotationErrorSamples : 0):F2} degrees");
            writer.WriteLine($"Maximum Rotation Deviation: {maxRotationDeviation:F2} degrees");
            writer.WriteLine($"Stability Violations: {stabilityViolations}");
            writer.WriteLine($"Total Contact Time: {cumulativeContactTime:F2} seconds");
            writer.WriteLine($"Contact Percentage: {(cumulativeContactTime/finalTaskTime*100):F2}%");
            
            writer.Close();
            writer = null;
            
            Debug.Log($"Stopped recording session: {sessionID}");
        }
    }

    void OnDisable()
    {
        if (isRecording)
        {
            StopRecording();
        }
    }

    // Public methods to access metrics
    public float GetTaskTime()
    {
        return currentTaskTime;
    }

    public Dictionary<string, float> GetCurrentMetrics()
    {
        return new Dictionary<string, float>
        {
            {"TaskTime", currentTaskTime},
            {"AverageRotationError", rotationErrorSamples > 0 ? totalRotationError / rotationErrorSamples : 0},
            {"MaxRotationDeviation", maxRotationDeviation},
            {"StabilityViolations", stabilityViolations}
        };
    }

    // Method to pause/resume recording if needed
    public void PauseRecording()
    {
        isRecording = false;
        Debug.Log("Recording paused at task time: " + currentTaskTime);
    }

    public void ResumeRecording()
    {
        if (writer != null)
        {
            isRecording = true;
            lastSampleTime = Time.time;
            Debug.Log("Recording resumed at task time: " + currentTaskTime);
        }
    }
}