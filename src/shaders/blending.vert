R"zzz(
#version 330 core
uniform vec4 light_position;
uniform vec3 camera_position;

uniform vec3 joint_trans[128];
uniform vec4 joint_rot[128];

in int jid0;
in int jid1;
in float w0;
in vec3 vector_from_joint0;
in vec3 vector_from_joint1;
in vec4 normal;
in vec2 uv;
in vec4 vert;

out vec4 vs_light_direction;
out vec4 vs_normal;
out vec2 vs_uv;
out vec4 vs_camera_direction;


mat2x4 QuatTrans2UDQ(vec4 q0, vec3 t)
{
	q0 = vec4(q0.w, q0.x, q0.y, q0.z);
   mat2x4 res;
   // non-dual part (just copy q0):
   for (int i=0; i<4; i++) res[0][i] = q0[i];
   // dual part:
   res[1][0] = -0.5*(t[0]*q0[1] + t[1]*q0[2] + t[2]*q0[3]);
   res[1][1] = 0.5*( t[0]*q0[0] + t[1]*q0[3] - t[2]*q0[2]);
   res[1][2] = 0.5*(-t[0]*q0[3] + t[1]*q0[0] + t[2]*q0[1]);
   res[1][3] = 0.5*( t[0]*q0[2] - t[1]*q0[1] + t[2]*q0[0]);
   return res;
}


vec3 qtransform(vec4 q, vec3 v) {
	return v + 2.0 * cross(cross(v, q.xyz) - q.w*v, q.xyz);
}

mat3x4 DQToMatrix(vec4 Qn, vec4 Qd)
{	
	mat3x4 M;
	float len2 = dot(Qn, Qn);
	float w = Qn.x, x = Qn.y, y = Qn.z, z = Qn.w;
	float t0 = Qd.x, t1 = Qd.y, t2 = Qd.z, t3 = Qd.w;
		
	M[0][0] = w*w + x*x - y*y - z*z; M[0][1] = 2*x*y - 2*w*z; M[0][2] = 2*x*z + 2*w*y;
	M[1][0] = 2*x*y + 2*w*z; M[1][1] = w*w + y*y - x*x - z*z; M[1][2] = 2*y*z - 2*w*x; 
	M[2][0] = 2*x*z - 2*w*y; M[2][1] = 2*y*z + 2*w*x; M[2][2] = w*w + z*z - x*x - y*y;
	
	M[0][3] = -2*t0*x + 2*w*t1 - 2*t2*z + 2*y*t3;
	M[1][3] = -2*t0*y + 2*t1*z - 2*x*t3 + 2*w*t2;
	M[2][3] = -2*t0*z + 2*x*t2 + 2*w*t3 - 2*t1*y;
	
	M /= len2;
	
	return M;	
}

// basic dual quaternion skinning:
vec4 dqs(vec4 position,vec4 dq0,vec4 dq1)
{
						
	
	
	mat3x4 M = DQToMatrix(dq0, dq1);
	vec3 oposition = position * M;
	return vec4(oposition, 1.0);
	
				
}

// per-vertex antipodality handling (this is the most robust, but not the most efficient way):
// outputs dqsAntipod(
// 			uniform float4x4 modelViewProj,
// 			uniform float4x4 modelViewIT,
// 			uniform float2x4 boneDQ[100])
// {
						
		
// 	float2x4 dq0 = boneDQ[IN.matrixIndices.x];
// 	float2x4 dq1 = boneDQ[IN.matrixIndices.y];
// 	float2x4 dq2 = boneDQ[IN.matrixIndices.z];
// 	float2x4 dq3 = boneDQ[IN.matrixIndices.w];

// 	if (dot(dq0[0], dq1[0]) < 0.0) dq1 *= -1.0;
// 	if (dot(dq0[0], dq2[0]) < 0.0) dq2 *= -1.0;	
// 	if (dot(dq0[0], dq3[0]) < 0.0) dq3 *= -1.0;
	
// 	float2x4 blendDQ = IN.weights.x*dq0;
// 	blendDQ += IN.weights.y*dq1;
// 	blendDQ += IN.weights.z*dq2;
// 	blendDQ += IN.weights.w*dq3;
	
// 	float3x4 M = DQToMatrix(blendDQ[0], blendDQ[1]);
// 	float3 position = mul(M, IN.position);
// 	float3 normal = mul(M, IN.normal);
		
// 	OUT.hPosition = mul(modelViewProj, float4(position, 1.0));
// 	OUT.hNormal = mul(modelViewIT, float4(normal, 0.0));
	
// 	return OUT;			
// }



void main() {
	// FIXME: Implement linear skinning here

	mat2x4 dq0 = QuatTrans2UDQ(joint_rot[jid0], joint_trans[jid0]);
	mat2x4 dq1 = QuatTrans2UDQ(joint_rot[jid1], joint_trans[jid1]);
	vec4 pos0 = dqs(vec4(vector_from_joint0, 1.0), dq0[0], dq0[1]);
	vec4 pos1 = dqs(vec4(vector_from_joint1, 1.0), dq1[0], dq1[1]);
	gl_Position = w0 * pos0 + (1 - w0) * pos1;
	// gl_Position = vert;
	vs_normal = normal;
	vs_light_direction = light_position - gl_Position;
	vs_camera_direction = vec4(camera_position, 1.0) - gl_Position;
	vs_uv = uv;
}
)zzz"