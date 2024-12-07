using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEngine.Assertions;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia
	Matrix4x4 I_ref_inv;						// inverse of reference inertia

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;					// for collision
	float friction		= 0.2f;					// for collision


	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
		
		I_ref_inv = I_ref.inverse;
		
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}

	Matrix4x4 MatrixAdd(Matrix4x4 A, Matrix4x4 B)
	{
		Matrix4x4 C = Matrix4x4.zero;
		for(int i=0;i<4;i++)
		{
			for(int j=0;j<4;j++)
			{
				C[i,j] = A[i,j] + B[i,j];
			}
		}
		return C;
	}
	
	Matrix4x4 MatrixSub(Matrix4x4 A, Matrix4x4 B)
	{
		Matrix4x4 C = Matrix4x4.zero;
		for(int i=0;i<4;i++)
		{
			for(int j=0;j<4;j++)
			{
				C[i,j] = A[i,j] - B[i,j];
			}
		}
		return C;
	}
	
	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		//计算所有在平面里面的点
		var mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;
		List<Vector3> in_plane = new List<Vector3>();

		var T = transform.position;
		var R = Matrix4x4.Rotate(transform.rotation);
		var I = R*I_ref*R.transpose;
		
		for(int i=0;i<vertices.Length;i++)
		{
			var pos = T+R.MultiplyPoint(vertices[i]);
			var vi = v+Vector3.Cross(w, pos-T);
			if(Vector3.Dot(pos-P, N)<0)
			{
				in_plane.Add(pos);
			}
		}
		
		if(in_plane.Count==0)
		{
			return;
		}
		
		Vector3 averagePoint = new Vector3(0, 0, 0);
		for(int i=0;i<in_plane.Count;i++)
		{
			averagePoint += in_plane[i];
		}
		averagePoint /= in_plane.Count;//世界坐标
		var relativePoint = averagePoint-T;//相对坐标
		
		var v_at_point = v+Vector3.Cross(w, relativePoint);
		
		if(Vector3.Dot(v_at_point, N)>0)
		{
			return;
		}
		
		Vector3 v_normal = Vector3.Dot(v_at_point, N)*N;
		Vector3 v_tangent = v_at_point-v_normal;
		float a =Mathf.Max(0,1-friction*(1+restitution)*v_normal.magnitude/v_tangent.magnitude);
		var v_new_normal = -restitution*v_normal;
		var v_new_tangent = a*v_tangent;
		var v_new = v_new_normal+v_new_tangent;
		
		var RriStarMatrix = Get_Cross_Matrix(relativePoint);
		var K = Matrix4x4.identity;
		K[0, 0] = K[1, 1] = K[2, 2] =K[3,3]=  1.0f / mass;
		K = MatrixSub(K, RriStarMatrix * I.inverse * RriStarMatrix);
		
		var J = K.inverse.MultiplyVector(v_new-v_at_point);
		
		v = v+1/mass*J;
		w = w + I.inverse.MultiplyVector(Vector3.Cross(relativePoint, J));

	}

	Quaternion QuaternionMul(Quaternion q1, Quaternion q2)
	{
		Quaternion q = new Quaternion();
		float s1 = q1.w;
		float s2 = q2.w;
		Vector3 v1 = new Vector3(q1.x, q1.y, q1.z);
		Vector3 v2 = new Vector3(q2.x, q2.y, q2.z);
		
		q.w = s1 * s2 - Vector3.Dot(v1, v2);
		var tmp = s1 * v2 + s2 * v1 + Vector3.Cross(v1, v2);
		q.x = tmp.x;
		q.y = tmp.y;
		q.z = tmp.z;
		
		return q;
	}
	
	Quaternion QuaternionAdd(Quaternion q1, Quaternion q2)
	{
		Quaternion q = new Quaternion();
		q.w = q1.w + q2.w;
		q.x = q1.x + q2.x;
		q.y = q1.y + q2.y;
		q.z = q1.z + q2.z;
		return q.normalized;
	}
	
	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			launched=true;
		}

		// Part I: Update velocities
		if (launched == false) return;

		//只受重力影响
		Vector3 g = new Vector3(0, -9.8f, 0);
		
		v+=g*dt;

		// Part II: Collision Impulse
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

		// Part III: Update position & orientation
		//Update linear status
		Vector3 x    = transform.position;
		//Update angular status
		Quaternion q = transform.rotation;

		v *= linear_decay;
		w*= angular_decay;
		
		x = x + v * dt;
		q = QuaternionAdd(q,QuaternionMul(new Quaternion(w.x*dt/2, w.y*dt/2, w.z*dt/2, 0), q));
		// Part IV: Assign to the object
		transform.position = x;
		transform.rotation = q;
	}
}
