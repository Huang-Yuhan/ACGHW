using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LastTransform : MonoBehaviour
{
    public Vector3 lastPosition;
    public Quaternion lastRotation;

    public Vector3 lastUpdateVelocity;

    public Vector3 lastUpdateAngularVelocity;

    IEnumerator updateCoroutine;



    private void Awake()
    {
        lastPosition = transform.position;
        lastRotation = transform.rotation;
        updateCoroutine = UpdateCoroutine();
        StartCoroutine(updateCoroutine);
    }

    IEnumerator UpdateCoroutine()
    {
        while (true)
        {
            yield return new WaitForFixedUpdate();
            // Debug.LogFormat("LastTransform: {0}", lastPosition);
            // Debug.LogFormat("NowTransform: {0}", transform.position);
            lastUpdateVelocity = (transform.position - lastPosition) / Time.fixedDeltaTime;
            lastUpdateAngularVelocity = CalculateAngularVelocityLargeRotation(
                lastRotation,
                transform.rotation,
                Time.fixedDeltaTime
            );
            lastPosition = transform.position;
            lastRotation = transform.rotation;

        }
    }

    /// <summary>
    /// 计算从上一帧到当前帧的角速度（弧度/秒）,注意：此方法使用小角度近似公式，需要单帧不要转动超过90度
    /// </summary>
    /// <param name="prevQuaternion">前一时刻的四元数</param>
    /// <param name="currentQuaternion">当前时刻的四元数</param>
    /// <param name="deltaTime">时间间隔（秒）</param>
    /// <returns>角速度向量（弧度/秒）</returns>
    /// 
    public Vector3 CalculateAngularVelocity(Quaternion prevQuaternion, Quaternion currentQuaternion, float deltaTime)
    {
        // 计算相对旋转四元数：Δq = currentQuaternion * prevQuaternion^-1
        Quaternion deltaQ = currentQuaternion * Quaternion.Inverse(prevQuaternion);

        // 小角度近似公式：角速度 ≈ 2 * (Δq的虚部) / Δt
        Vector3 angularVelocityRadPerSec = new Vector3(
            deltaQ.x, deltaQ.y, deltaQ.z
        ) * (2.0f / deltaTime);

        return angularVelocityRadPerSec;
    }
    
    /// <summary>
    /// 计算大角度旋转的角速度（弧度/秒）
    /// </summary>
    /// <param name="prevQuaternion">前一帧的四元数</param>
    /// <param name="currentQuaternion">当前帧的四元数</param>
    /// <param name="deltaTime">时间间隔（秒）</param>
    /// <returns>角速度向量（弧度/秒）</returns>
    public Vector3 CalculateAngularVelocityLargeRotation(
        Quaternion prevQuaternion, 
        Quaternion currentQuaternion, 
        float deltaTime)
    {
        // 1. 计算相对旋转四元数 Δq
        Quaternion deltaQ = currentQuaternion * Quaternion.Inverse(prevQuaternion);

        // 2. 将 Δq 转换为轴角表示（角度 + 旋转轴）
        deltaQ.ToAngleAxis(out float angleDegrees, out Vector3 rotationAxis);

        // 3. 处理角度周期性问题（确保角度在 [0, 360] 范围内）
        angleDegrees = Mathf.Repeat(angleDegrees, 360.0f);

        // 4. 转换为弧度并计算角速度
        float angleRadians = angleDegrees * Mathf.Deg2Rad;
        Vector3 angularVelocityRadPerSec = (angleRadians / deltaTime) * rotationAxis.normalized;

        return angularVelocityRadPerSec;
    }
 
}
