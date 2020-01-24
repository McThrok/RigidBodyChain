#pragma once
#include "AdapterReader.h"
#include "Shaders.h"
#include "Vertex.h"
#include <SpriteBatch.h>
#include <SpriteFont.h>
#include <iostream>
#include <fstream>
#include <WICTextureLoader.h>
#include "VertexBuffer.h"
#include "IndexBuffer.h"
#include "ConstantBuffer.h"
#include "Camera.h"
#include "..\\Timer.h"
#include "MyImGui.h"
#include "..\\Simulation.h"
#include "ImGUI\\imgui.h"
#include "ImGUI\\imgui_impl_win32.h"
#include "ImGUI\\imgui_impl_dx11.h"
#include "..\\GuiData.h"

using namespace std;
using Microsoft::WRL::ComPtr;

class Graphics
{
public:
	bool Initialize(HWND hwnd, int width, int height);
	void RenderFrame();
	Camera camera;
	Simulation* simulation;
	GuiData* guiData;

private:
	bool InitializeDirectX(HWND hwnd);
	bool InitializeShaders();


	void InitGui(HWND hwnd);
	void RendeGui();
	void RenderMainPanel();
	void RenderVisualisation();

	bool InitializeScene();

	ComPtr<ID3D11SamplerState> sampler;


	ComPtr<ID3D11Device> device;
	ComPtr<ID3D11DeviceContext> deviceContext;
	ComPtr<IDXGISwapChain> swapchain;
	ComPtr<ID3D11RenderTargetView> renderTargetView;

	VertexShader vertexshader, deformationShader;
	PixelShader pixelshader, pureColorPixelshader;
	GeometryShader normalsShader;

	Vector3 pos;
	ConstantBuffer<ColoredObjectBuffer> cbColoredObject;
	VertexBuffer<VertexPN>  vbCube, vbGround;
	IndexBuffer ibCube, ibGround;

	ComPtr<ID3D11DepthStencilView> depthStencilView;
	ComPtr<ID3D11Texture2D> depthStencilBuffer;
	ComPtr<ID3D11DepthStencilState> depthStencilState;
	ComPtr<ID3D11RasterizerState> rasterizerState;

	int windowWidth = 0;
	int windowHeight = 0;
	Timer fpsTimer;
};