using UnityEngine;
using System.Collections.Generic;
using System;

[Serializable]
public class HapticInteraction
{
    public float startTime;          // When the interaction started
    public float endTime;            // When the interaction ended
    public float duration;           // How long the interaction lasted
    public float maxForce;           // Maximum force during interaction
    public float averageForce;       // Average force during interaction
    public Vector3 contactPoint;     // Where the interaction occurred
    public string objectName;        // Name of the interacted object
    public List<float> forceValues;  // List of forces during interaction

    public HapticInteraction()
    {
        forceValues = new List<float>();
    }

    public void Initialize()
    {
        startTime = Time.time;
        maxForce = 0f;
        averageForce = 0f;
    }
}

public class HapticInteractionTracker : MonoBehaviour
{
    private HapticPlugin hapticPlugin;
    private bool isInteracting = false;
    private HapticInteraction currentInteraction;

    [SerializeField]
    private List<HapticInteraction> interactionHistory = new List<HapticInteraction>();
    public int totalInteractions { get; private set; }

    // Settings for data collection
    public bool saveToFile = false;
    public string filePrefix = "haptic_interaction_";
    public float forceSamplingRate = 0.016f; // Sample force every 16ms by default
    private float lastForceSampleTime = 0f;

    private void Start()
    {
        // Find the HapticPlugin in the scene
        hapticPlugin = FindObjectOfType<HapticPlugin>();
        if (hapticPlugin == null)
        {
            Debug.LogError("No HapticPlugin found in the scene!");
            enabled = false;
            return;
        }

        totalInteractions = 0;

        // Subscribe to the haptic events
        if (hapticPlugin.Events != null)
        {
            hapticPlugin.Events.OnTouch.AddListener(OnHapticTouch);
        }
    }

    private void OnHapticTouch()
    {
        if (!isInteracting)
        {
            StartNewInteraction();
        }
    }

    private void StartNewInteraction()
    {
        isInteracting = true;
        currentInteraction = new HapticInteraction();
        currentInteraction.Initialize();
        currentInteraction.contactPoint = hapticPlugin.LastContact;

        // Try to get the name of the object being touched
        if (hapticPlugin.ContactPointsInfo.Count > 0)
        {
            currentInteraction.objectName = hapticPlugin.ContactPointsInfo[0].ColliderName;
        }
    }

    private void EndCurrentInteraction()
    {
        if (currentInteraction != null)
        {
            currentInteraction.endTime = Time.time;
            currentInteraction.duration = currentInteraction.endTime - currentInteraction.startTime;
            
            // Calculate average force
            if (currentInteraction.forceValues.Count > 0)
            {
                float sum = 0f;
                foreach (float force in currentInteraction.forceValues)
                {
                    sum += force;
                }
                currentInteraction.averageForce = sum / currentInteraction.forceValues.Count;
            }

            interactionHistory.Add(currentInteraction);
            totalInteractions++;

            if (saveToFile)
            {
                SaveInteractionToFile(currentInteraction);
            }
        }

        isInteracting = false;
        currentInteraction = null;
    }

    private void Update()
    {
        if (!hapticPlugin) return;

        // Check if we're still touching
        if (isInteracting)
        {
            // Check if we're no longer touching based on the haptic plugin's state
            if (hapticPlugin.ContactPointsInfo.Count == 0)
            {
                EndCurrentInteraction();
            }
            else
            {
                // Sample force if enough time has passed
                if (Time.time - lastForceSampleTime >= forceSamplingRate)
                {
                    float currentForce = hapticPlugin.MagForce;
                    currentInteraction.forceValues.Add(currentForce);

                    // Update max force if necessary
                    if (currentForce > currentInteraction.maxForce)
                    {
                        currentInteraction.maxForce = currentForce;
                    }

                    lastForceSampleTime = Time.time;
                }
            }
        }
    }

    private void SaveInteractionToFile(HapticInteraction interaction)
    {
        string filename = $"{filePrefix}{DateTime.Now:yyyyMMdd_HHmmss}.csv";
        string path = System.IO.Path.Combine(Application.persistentDataPath, filename);

        using (System.IO.StreamWriter writer = new System.IO.StreamWriter(path, true))
        {
            // Write header if this is a new file
            if (new System.IO.FileInfo(path).Length == 0)
            {
                writer.WriteLine("Object Name,Start Time,End Time,Duration,Max Force,Average Force,Contact Point X,Contact Point Y,Contact Point Z,Force Values");
            }

            // Write interaction data
            string forceValues = string.Join(";", interaction.forceValues);
            writer.WriteLine($"{interaction.objectName},{interaction.startTime},{interaction.endTime},{interaction.duration}," +
                           $"{interaction.maxForce},{interaction.averageForce}," +
                           $"{interaction.contactPoint.x},{interaction.contactPoint.y},{interaction.contactPoint.z}," +
                           $"{forceValues}");
        }
    }

    // Public methods to access interaction data
    public float GetAverageInteractionDuration()
    {
        if (interactionHistory.Count == 0) return 0f;

        float totalDuration = 0f;
        foreach (var interaction in interactionHistory)
        {
            totalDuration += interaction.duration;
        }
        return totalDuration / interactionHistory.Count;
    }

    public float GetAverageMaxForce()
    {
        if (interactionHistory.Count == 0) return 0f;

        float totalMaxForce = 0f;
        foreach (var interaction in interactionHistory)
        {
            totalMaxForce += interaction.maxForce;
        }
        return totalMaxForce / interactionHistory.Count;
    }

    public void ClearInteractionHistory()
    {
        interactionHistory.Clear();
        totalInteractions = 0;
    }

    public List<HapticInteraction> GetInteractionHistory()
    {
        return new List<HapticInteraction>(interactionHistory);
    }

    private void OnDisable()
    {
        if (hapticPlugin?.Events != null)
        {
            hapticPlugin.Events.OnTouch.RemoveListener(OnHapticTouch);
        }
    }
}