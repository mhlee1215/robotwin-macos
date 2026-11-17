#include <metal_stdlib>
using namespace metal;

float4x4 rotation(float3 a, float q) {
    float c=cos(q), s=sin(q), t=1-c;
    return float4x4(float4(c+a.x*a.x*t,a.y*a.x*t+a.z*s,a.z*a.x*t-a.y*s,0),
                    float4(a.x*a.y*t-a.z*s,c+a.y*a.y*t,a.z*a.y*t+a.x*s,0),
                    float4(a.x*a.z*t+a.y*s,a.y*a.z*t-a.x*s,c+a.z*a.z*t,0),float4(0,0,0,1));
}
float4x4 fk(device const float4x4 *origins, device const float4 *axes,
            thread const float *q, thread float3 *positions, thread float3 *directions,
            thread float4x4 *links) {
    float4x4 T(1);
    for (uint j=0;j<6;++j) {
        T=T*origins[j];
        positions[j]=T[3].xyz;
        directions[j]=(T*float4(axes[j].xyz,0)).xyz;
        T=T*rotation(axes[j].xyz,q[j]);
        links[j]=T;
    }
    return T;
}
float3 log_rotation(float3x3 R) {
    // Quaternion extraction remains well conditioned for near-pi errors.
    float4 q;
    float tr=R[0][0]+R[1][1]+R[2][2];
    if (tr>0) {
        float s=2*sqrt(tr+1);
        q=float4((R[1][2]-R[2][1])/s,(R[2][0]-R[0][2])/s,(R[0][1]-R[1][0])/s,s/4);
    } else if (R[0][0]>R[1][1] && R[0][0]>R[2][2]) {
        float s=2*sqrt(max(1+R[0][0]-R[1][1]-R[2][2],1e-12f));
        q=float4(s/4,(R[1][0]+R[0][1])/s,(R[2][0]+R[0][2])/s,(R[1][2]-R[2][1])/s);
    } else if (R[1][1]>R[2][2]) {
        float s=2*sqrt(max(1+R[1][1]-R[0][0]-R[2][2],1e-12f));
        q=float4((R[1][0]+R[0][1])/s,s/4,(R[2][1]+R[1][2])/s,(R[2][0]-R[0][2])/s);
    } else {
        float s=2*sqrt(max(1+R[2][2]-R[0][0]-R[1][1],1e-12f));
        q=float4((R[2][0]+R[0][2])/s,(R[2][1]+R[1][2])/s,s/4,(R[0][1]-R[1][0])/s);
    }
    if(q.w<0)q=-q;
    float n=length(q.xyz);
    return q.xyz*(2*atan2(n,max(q.w,0.f))/max(n,1e-8f));
}
void pose_error(float4x4 T, float4x4 target, thread float *e) {
    float3 p=target[3].xyz-T[3].xyz;
    float3 w=log_rotation(float3x3(target[0].xyz,target[1].xyz,target[2].xyz)*transpose(float3x3(T[0].xyz,T[1].xyz,T[2].xyz)));
    for(uint i=0;i<3;++i){ e[i]=p[i]; e[i+3]=w[i]; }
}
float norm6(thread const float *e) { float v=0;for(uint j=0;j<6;++j)v+=e[j]*e[j];return v; }
kernel void forward_kinematics(device const float4x4 *origins [[buffer(0)]],
 device const float4 *axes [[buffer(1)]], device const float *configs [[buffer(2)]],
 device float4x4 *out [[buffer(3)]], uint id [[thread_position_in_grid]]) {
    float q[6];for(uint j=0;j<6;++j)q[j]=configs[id*6+j];
    float3 p[6],a[6];float4x4 links[6];out[id]=fk(origins,axes,q,p,a,links);
}
kernel void inverse_kinematics(device const float4x4 *origins [[buffer(0)]],
 device const float4 *axes [[buffer(1)]], device const float2 *limits [[buffer(2)]],
 device const float *seeds [[buffer(3)]], device const float4x4 *target [[buffer(4)]],
 device const uint *iterations [[buffer(5)]], device float *out [[buffer(6)]], uint id [[thread_position_in_grid]]) {
    float q[6];for(uint j=0;j<6;++j)q[j]=clamp(seeds[id*6+j],limits[j].x,limits[j].y);
    float damp=.02f;
    for(uint iter=0;iter<iterations[0];++iter){
        float3 p[6],axesWorld[6];float4x4 links[6];
        float4x4 T=fk(origins,axes,q,p,axesWorld,links);
        float e[6];pose_error(T,target[0],e);
        if(length(float3(e[0],e[1],e[2]))<.00015f && length(float3(e[3],e[4],e[5]))<.001f)break;
        float J[36];
        for(uint j=0;j<6;++j){float3 v=cross(axesWorld[j],T[3].xyz-p[j]);for(uint k=0;k<3;++k){J[k*6+j]=v[k];J[(k+3)*6+j]=axesWorld[j][k];}}
        float A[42];
        for(uint i=0;i<6;++i){
            for(uint j=0;j<6;++j){float v=0;for(uint k=0;k<6;++k)v+=J[k*6+i]*J[k*6+j];A[i*7+j]=v+(i==j?damp*damp:0);}
            float v=0;for(uint k=0;k<6;++k)v+=J[k*6+i]*e[k];A[i*7+6]=v;
        }
        // Pivoted elimination runs entirely on the GPU, no MPS CPU fallback.
        for(uint k=0;k<6;++k){
            uint pivot=k;for(uint i=k+1;i<6;++i)if(abs(A[i*7+k])>abs(A[pivot*7+k]))pivot=i;
            for(uint j=k;j<7;++j){float t=A[k*7+j];A[k*7+j]=A[pivot*7+j];A[pivot*7+j]=t;}
            float v=A[k*7+k];if(abs(v)<1e-9f)v=1e-9f;
            for(uint j=k;j<7;++j)A[k*7+j]/=v;
            for(uint i=0;i<6;++i)if(i!=k){float f=A[i*7+k];for(uint j=k;j<7;++j)A[i*7+j]-=f*A[k*7+j];}
        }
        float largest=0;for(uint j=0;j<6;++j)largest=max(largest,abs(A[j*7+6]));
        float candidate[6];for(uint j=0;j<6;++j)candidate[j]=clamp(q[j]+A[j*7+6]*min(1.f,.25f/max(largest,1e-8f)),limits[j].x,limits[j].y);
        float next[6];pose_error(fk(origins,axes,candidate,p,axesWorld,links),target[0],next);
        if(norm6(next)<norm6(e)){for(uint j=0;j<6;++j)q[j]=candidate[j];damp=max(.002f,damp*.7f);}
        else damp=min(1.f,damp*2.f);
    }
    float3 p[6],a[6];float4x4 links[6];float e[6];pose_error(fk(origins,axes,q,p,a,links),target[0],e);
    for(uint j=0;j<6;++j)out[id*8+j]=q[j];
    out[id*8+6]=length(float3(e[0],e[1],e[2]));out[id*8+7]=length(float3(e[3],e[4],e[5]));
}
kernel void collisions(device const float4x4 *origins [[buffer(0)]], device const float4 *axes [[buffer(1)]],
 device const float4 *spheres [[buffer(2)]], device const int *attached [[buffer(3)]],
 device const uint2 *pairs [[buffer(4)]], device const float4x4 *boxes [[buffer(5)]],
 device const float4 *halfsize [[buffer(6)]], device const uint *counts [[buffer(7)]],
 device const float *configs [[buffer(8)]], device uint *out [[buffer(9)]], uint id [[thread_position_in_grid]]) {
    float q[6];for(uint j=0;j<6;++j)q[j]=configs[id*6+j];
    float3 p[6],a[6];float4x4 links[6];fk(origins,axes,q,p,a,links);
    float3 centers[128];
    for(uint i=0;i<counts[0];++i)centers[i]=attached[i]<0?spheres[i].xyz:(links[attached[i]]*float4(spheres[i].xyz,1)).xyz;
    for(uint k=0;k<counts[1];++k){uint2 pair=pairs[k];if(distance(centers[pair.x],centers[pair.y])<spheres[pair.x].w+spheres[pair.y].w){out[id]=k+1;return;}}
    for(uint b=0;b<counts[2];++b)for(uint i=0;i<counts[0];++i){
        float3 local=(boxes[b]*float4(centers[i],1)).xyz;
        if(distance(local,clamp(local,-halfsize[b].xyz,halfsize[b].xyz))<spheres[i].w+halfsize[b].w){out[id]=counts[1]+b*counts[0]+i+1;return;}
    }
    out[id]=0;
}
kernel void quintic(device const float *starts [[buffer(0)]], device const float *ends [[buffer(1)]],
 device const float4 *samples [[buffer(2)]], device float *out [[buffer(3)]], uint id [[thread_position_in_grid]]) {
    uint seg=uint(samples[id].x);float u=samples[id].y,t=samples[id].z;
    float s=10*pow(u,3.f)-15*pow(u,4.f)+6*pow(u,5.f);
    float v=(30*u*u-60*u*u*u+30*pow(u,4.f))/t;
    float a=(60*u-180*u*u+120*u*u*u)/(t*t);
    for(uint j=0;j<6;++j){float delta=ends[seg*6+j]-starts[seg*6+j];out[id*18+j]=starts[seg*6+j]+s*delta;out[id*18+6+j]=v*delta;out[id*18+12+j]=a*delta;}
}
kernel void farthest_points(device const float *points [[buffer(0)]], device const uint *counts [[buffer(1)]],
 device float *distances [[buffer(2)]], device uint *output [[buffer(3)]], uint tid [[thread_index_in_threadgroup]]) {
    threadgroup float maxima[256];threadgroup uint indices[256];threadgroup uint selected;
    if(tid==0)selected=0;
    for(uint i=tid;i<counts[0];i+=256)distances[i]=INFINITY;
    threadgroup_barrier(mem_flags::mem_device|mem_flags::mem_threadgroup);
    for(uint k=0;k<counts[1];++k){
        uint chosen=selected;if(tid==0)output[k]=chosen;
        float3 p=float3(points[chosen*3],points[chosen*3+1],points[chosen*3+2]);
        float best=-1;uint bestIndex=0;
        for(uint i=tid;i<counts[0];i+=256){float3 d=float3(points[i*3],points[i*3+1],points[i*3+2])-p;float value=min(distances[i],dot(d,d));distances[i]=value;if(value>best||(value==best&&i<bestIndex)){best=value;bestIndex=i;}}
        maxima[tid]=best;indices[tid]=bestIndex;threadgroup_barrier(mem_flags::mem_threadgroup);
        for(uint stride=128;stride>0;stride/=2){if(tid<stride&&(maxima[tid+stride]>maxima[tid]||(maxima[tid+stride]==maxima[tid]&&indices[tid+stride]<indices[tid]))){maxima[tid]=maxima[tid+stride];indices[tid]=indices[tid+stride];}threadgroup_barrier(mem_flags::mem_threadgroup);}
        if(tid==0)selected=indices[0];threadgroup_barrier(mem_flags::mem_device|mem_flags::mem_threadgroup);
    }
}
