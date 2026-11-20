// Shared CPU/CUDA sampling rules. Only the GPU marks a packed map concurrently.
#ifdef __CUDACC__
#define RAY_INLINE __device__ inline
#else
#define RAY_INLINE inline
#endif
template<class Masks>
RAY_INLINE void trace_ray(const double* ray, int count, const double* g,
                          int w, int h, int bins, Masks masks) {
    // g: sensor xyz, grid origin xy, cos/sin(yaw), resolution,
    // plane a/b/c, obstacle height, maximum height, bin height.
    for (int i = 0; i <= count; ++i) {
        double t = double(i) / count;
        double x = g[0] + ray[0]*t, y = g[1] + ray[1]*t, z = g[2] + ray[2]*t;
        double dx = x-g[3], dy = y-g[4];
        double u = (g[5]*dx+g[6]*dy)/g[7], v = (-g[6]*dx+g[5]*dy)/g[7];
        if (!(u >= 0 && u < w && v >= 0 && v < h)) continue;
        int cell = int(v)*w+int(u);
        double height = z-(g[8]*x+g[9]*y+g[10]);
        if (height >= -.08 && height <= g[11]) masks.mark(0,cell);
        if (height >= g[11] && height < g[12]) {
            int level = int((height-g[11])/g[13]);
            if (level < bins) masks.mark(1,level*w*h+cell);
        }
    }
}
#ifdef __CUDACC__
struct DeviceMasks {
    unsigned* low; unsigned* mid;
    __device__ void mark(int which, int id) {
        atomicOr((which ? mid : low) + (id >> 5), 1u << (id & 31));
    }
};
extern "C" __global__ void pm01_trace(const double* rays, const int* counts,
    const double* g, unsigned* low, unsigned* mid, int n, int w, int h, int bins) {
    int r = blockIdx.x*blockDim.x+threadIdx.x;
    if (r < n) trace_ray(rays+3*r,counts[r],g,w,h,bins,DeviceMasks{low,mid});
}
#endif
#undef RAY_INLINE
