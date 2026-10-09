add_runtime_sample(MetalCubeSample
	MetalCubeSample.cpp
	CubeRenderSystem.cpp
	CubeRotationSystem.cpp
)
embed_sample_shader(MetalCubeSample metal.vert kMetalVertexShader)
embed_sample_shader(MetalCubeSample metal.frag kMetalFragmentShader)
