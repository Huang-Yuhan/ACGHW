using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class ShowFPs : MonoBehaviour
{
    private TextMeshProUGUI text;
    
    private void Start()
    {
        text = GetComponent<TextMeshProUGUI>();
    }
    
    private void Update()
    {
        text.text = $"FPS: {1 / Time.deltaTime}";
    }
}
