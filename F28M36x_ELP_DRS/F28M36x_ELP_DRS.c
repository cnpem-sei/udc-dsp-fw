#include "F28M36x_ELP_DRS.h"


void InitBuffer(tBuffer *ptrBuff, volatile float *ptrStart, Uint16 size)
{
	ptrBuff->BufferBusy = Buffer_Idle;
	ptrBuff->PtrBufferStart = ptrStart;
	ptrBuff->PtrBufferEnd = ptrStart + size - 1;
	ResetBuffer(ptrBuff);
}

void ResetBuffer(tBuffer *ptrBuff)
{
	ptrBuff->PtrBufferK = ptrBuff->PtrBufferStart;

	while(ptrBuff->PtrBufferK < ptrBuff->PtrBufferEnd)
	{
		*(ptrBuff->PtrBufferK++) = 0.0;
	}

	ptrBuff->PtrBufferK = ptrBuff->PtrBufferStart;
}

Uint16 SizeBuffer(tBuffer *ptrBuff)
{
	Uint16 size;
	size = ptrBuff->PtrBufferStart - ptrBuff->PtrBufferEnd;
	return size;
}

Uint16 IndexBuffer(tBuffer *ptrBuff)
{
	Uint16 idx;
	idx = ptrBuff->PtrBufferK - ptrBuff->PtrBufferStart;
	return idx;
}

Uint16 WriteBuffer(tBuffer *ptrBuff, float data)
{
	if(ptrBuff->BufferBusy)
	{
		*(ptrBuff->PtrBufferK) = data;
		if(ptrBuff->PtrBufferK++ >= ptrBuff->PtrBufferEnd)
		{
			ptrBuff->PtrBufferK = ptrBuff->PtrBufferStart;
			return 1;
		}
		else
		{
			return 0;
		}
	}
}
