struct Input { @location(0) position: vec3<f32>, @location(1) color: vec4<f32> };
struct Output { @builtin(position) position: vec4<f32>, @location(0) color: vec4<f32> };
@vertex fn vs_main(v: Input) -> Output {
    var o: Output;
    o.position = vec4<f32>(v.position, 1.0);
    o.color = v.color;
    return o;
}
@fragment fn fs_main(v: Output) -> @location(0) vec4<f32> { return v.color; }
