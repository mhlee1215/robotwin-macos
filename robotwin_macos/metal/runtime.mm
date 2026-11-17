#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#include <cstring>
#include <cstdio>
#include <algorithm>

struct Context {
    id<MTLDevice> device;
    id<MTLCommandQueue> queue;
    id<MTLLibrary> library;
    NSMutableDictionary *pipelines;
};
static void report(char *dst, const char *msg) { snprintf(dst, 4096, "%s", msg); }
extern "C" void *metal_create(const char *source, char *error) {
    @autoreleasepool {
        auto ctx = new Context();
        ctx->device = MTLCreateSystemDefaultDevice();
        if (!ctx->device) { report(error, "No Metal GPU"); delete ctx; return nullptr; }
        ctx->queue = [ctx->device newCommandQueue];
        NSError *err = nil;
        MTLCompileOptions *options = [MTLCompileOptions new];
        options.fastMathEnabled = NO;
        ctx->library = [ctx->device newLibraryWithSource:[NSString stringWithUTF8String:source] options:options error:&err];
        if (!ctx->library) { report(error, err.localizedDescription.UTF8String); delete ctx; return nullptr; }
        ctx->pipelines = [NSMutableDictionary new];
        return ctx;
    }
}
extern "C" const char *metal_device(void *handle) {
    return ((Context *)handle)->device.name.UTF8String;
}
extern "C" int metal_run(void *handle, const char *name, unsigned count,
                         void **pointers, size_t *sizes, unsigned nbuf,
                         double *seconds, char *error) {
    @autoreleasepool {
        auto ctx = (Context *)handle;
        NSString *key = [NSString stringWithUTF8String:name];
        id<MTLComputePipelineState> pipeline = ctx->pipelines[key];
        if (!pipeline) {
            NSError *err = nil;
            id<MTLFunction> function = [ctx->library newFunctionWithName:key];
            if (!function) { report(error, "Unknown Metal kernel"); return 1; }
            pipeline = [ctx->device newComputePipelineStateWithFunction:function error:&err];
            if (!pipeline) { report(error, err.localizedDescription.UTF8String); return 1; }
            ctx->pipelines[key] = pipeline;
        }
        NSMutableArray *buffers = [NSMutableArray new];
        id<MTLCommandBuffer> command = [ctx->queue commandBuffer];
        id<MTLComputeCommandEncoder> encoder = [command computeCommandEncoder];
        [encoder setComputePipelineState:pipeline];
        for (unsigned i = 0; i < nbuf; ++i) {
            id<MTLBuffer> buffer = [ctx->device newBufferWithBytes:pointers[i] length:sizes[i] options:MTLResourceStorageModeShared];
            if (!buffer) { report(error, "Metal buffer allocation failed"); return 1; }
            [buffers addObject:buffer];
            [encoder setBuffer:buffer offset:0 atIndex:i];
        }
        NSUInteger width = std::min((NSUInteger)256, pipeline.maxTotalThreadsPerThreadgroup);
        if (strcmp(name, "farthest_points") == 0) {
            // One cooperative group performs the sequential FPS decisions.
            width = 256;
            if (pipeline.maxTotalThreadsPerThreadgroup < width) { report(error, "FPS needs 256 threads"); return 1; }
            [encoder dispatchThreadgroups:MTLSizeMake(1,1,1) threadsPerThreadgroup:MTLSizeMake(width,1,1)];
        } else {
            [encoder dispatchThreads:MTLSizeMake(count,1,1) threadsPerThreadgroup:MTLSizeMake(width,1,1)];
        }
        [encoder endEncoding];
        [command commit];
        [command waitUntilCompleted];
        if (command.status == MTLCommandBufferStatusError) { report(error, command.error.localizedDescription.UTF8String); return 1; }
        *seconds = command.GPUEndTime-command.GPUStartTime;
        id<MTLBuffer> output = buffers[nbuf-1];
        memcpy(pointers[nbuf-1], output.contents, sizes[nbuf-1]);
        return 0;
    }
}
extern "C" void metal_destroy(void *handle) { delete (Context *)handle; }
