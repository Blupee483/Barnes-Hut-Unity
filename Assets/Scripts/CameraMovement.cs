using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraMovement : MonoBehaviour
{
    [Header("Drag settings")]
    [SerializeField] private float mouseMoveDamping = 0.1f;
    [Header("Zoom settings")]
    [SerializeField] private float minZoom = 2f;
    [SerializeField] private float maxZoom = 80f;
    [SerializeField] private float zoomSensitivity = 8f;
    [SerializeField] private float zoomSmoothness = 10f;

    private Camera cam;
    private float targetZoom;

    private Vector3 prevMousePos;
    private int buffer = 0;

    void Start()
    {
        cam = GetComponent<Camera>();
        targetZoom = cam.orthographicSize;
    }
    void Update()
    {
        //mouse drag movement
        if (Input.GetKey(KeyCode.Mouse0))
        {
            if(buffer < 0) prevMousePos = Input.mousePosition;
            buffer = 1;
            transform.position += (-Input.mousePosition + prevMousePos) * mouseMoveDamping;
        }
        else
        {
            buffer = -1;
        }
        prevMousePos = Input.mousePosition;

        //zoom with scrollwheel
        float scrollInput = Input.GetAxis("Mouse ScrollWheel");

        if(Mathf.Abs(scrollInput) > 0.001f)
        {
            //scrollInput = Mathf.Sign(scrollInput);
            Vector3 mouseWorldPosBeforeZoom = cam.ScreenToWorldPoint(Input.mousePosition);

            targetZoom -= scrollInput * zoomSensitivity;
            targetZoom = Mathf.Clamp(targetZoom, minZoom, maxZoom);

            cam.orthographicSize = targetZoom;

            Vector3 mouseWorldPosAfterZoom = cam.ScreenToWorldPoint(Input.mousePosition);
            Vector3 positionOffset = mouseWorldPosBeforeZoom - mouseWorldPosAfterZoom;

            Vector3 targetPosition = transform.position + positionOffset;
            targetPosition.z = transform.position.z;

            transform.position = targetPosition;
        }

        if (Mathf.Abs(cam.orthographicSize - targetZoom) > 0.01f)
        {
            cam.orthographicSize = Mathf.Lerp(cam.orthographicSize, targetZoom, Time.deltaTime * zoomSmoothness);
        }

        mouseMoveDamping = cam.orthographicSize / 200;
    }
}

