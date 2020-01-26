#pragma once
#include <DirectXMath.h>

struct ColoredObjectBuffer
{
	DirectX::XMMATRIX wvpMatrix;
	DirectX::XMMATRIX worldMatrix;
	DirectX::XMMATRIX invWorldMatrix;
	DirectX::XMVECTOR color;
};


struct LightBuffer
{
	DirectX::XMVECTOR position;
	DirectX::XMVECTOR color;
};