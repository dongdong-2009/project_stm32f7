/*=====================================================================================
 File name:        InitPosDet.c  (IQ version)
                    
 Originator:	DL.K

 Description:  The Initial Position Detection
=====================================================================================
 History:
-------------------------------------------------------------------------------------
-------------------------------------------------------------------------------------*/

#include "modules/InitPosDet/InitPosDet.h"


INITPOSDET_Handle INITPOSDET_Init(void *pMemory, const size_t numBytes)//! \Mod By Dl.K
{
	INITPOSDET_Handle handle;
	uint16_t i;

	if (numBytes < sizeof(INITPOSDET_Obj))
		return((INITPOSDET_Handle)NULL);

	// assign the handle
	handle = (INITPOSDET_Handle)pMemory;

	handle->InitPosDetFlag = true;
	handle->InitPosAngleCount = 0;
	handle->InitPosAngle = (0.0);
	handle->SubdivideAngle = (SUBDIVIDE_ANGLE_PU);
	handle->VolVecAngleFlag = false;
	handle->VolVecAngleCount = 0;
	handle->VolVecAngle = (0.0);
	handle->VolVecAngleOffset = (-0.5);//-0.1666667
	handle->VolVecAngleIntervalTime = VOL_VEC_ANGLE_INTERVAL_TIME_SCLAR;
	handle->IdSampAngleFlag = false;
	handle->IdSampAngle = (0.0);
	handle->IdSampAngleCount = 0;
	for (i = 0; i<VOL_VEC_ANGLE_SCLAR / 2; i++)
	{
		handle->IdEvg[i] = (0.0);
	}
	handle->IdEvgCoeff = (1.0 / ID_EVG_SCLAR1);
	handle->IdEvgCount = 0;

	handle->IdEvgTmp = (0.0);

	return(handle);
}


void BubbleSort(float_t a[], uint16_t b[],uint16_t size)
{
	uint16_t lastSwapPos = 0,lastSwapPosTemp = 0;
	uint16_t i,j;
	float_t temp;
	uint16_t temp1;
    for (i = 0; i < size - 1; i++)
    {
        lastSwapPos = lastSwapPosTemp;
        for (j = size - 1; j >lastSwapPos; j--)
        {
            if (a[j - 1] < a[j])
            {
            	temp = a[j - 1];
                a[j - 1] = a[j];
                a[j] = temp;

                temp1 = b[j - 1];
                b[j - 1] = b[j];
                b[j] = temp1;

                lastSwapPosTemp = j;
            }
        }
        if (lastSwapPos == lastSwapPosTemp)break;
    }
}

uint16_t MATH_UINT16_T_ABS(uint16_t a, uint16_t b)
{
	if(a>b)
		return a-b;
	else
		return b-a;
}

void INITPOSDET_Run(INITPOSDET_Handle handle, float_t value)
{
	INITPOSDET_Obj *obj = (INITPOSDET_Obj *)handle;
	uint16_t i,InitPosAngleCount_Evg,InitPosAngleCount_Min,InitPosAngleCount_Mid,InitPosAngleCount_Max,b[VOL_VEC_ANGLE_SCLAR/2]={0};

	if(obj->IdSampAngleFlag)
	{
		if( obj->IdEvgCount<ID_EVG_SCLAR1*VOL_VEC_ANGLE_SCLAR/2 )
		{
			if(obj->IdEvgCount>=ID_EVG_SCLAR2*VOL_VEC_ANGLE_SCLAR/2)// Need Improve
			{
				float_t value_tmp = obj->IdEvg[obj->IdSampAngleCount]/(obj->IdEvgCount/(VOL_VEC_ANGLE_SCLAR/2));
				if( MATH_abs( value_tmp - (obj->IdEvgCoeff * value) )
						> ((0.2f) * value_tmp) )
				{
					obj->IdEvg[obj->IdSampAngleCount] += value_tmp;
				}
				else
				{
					obj->IdEvg[obj->IdSampAngleCount] += (obj->IdEvgCoeff * value);
				}
			}
			else
			{
				obj->IdEvg[obj->IdSampAngleCount] += (obj->IdEvgCoeff * value);
			}

			obj->IdEvgCount++;

			if( obj->IdEvgCount>=ID_EVG_SCLAR1*VOL_VEC_ANGLE_SCLAR/2 )
			{
//				Id_max = obj->IdEvg[0];
//				obj->InitPosAngleCount = 0;
//				for(i=0;i<VOL_VEC_ANGLE_SCLAR/2;i++)
//				{
//					if( Id_max < obj->IdEvg[i] )
//					{
//						Id_max = obj->IdEvg[i];
//						obj->InitPosAngleCount = i;
//					}
//				}

				for(i=0;i<VOL_VEC_ANGLE_SCLAR/2;i++)
				{
					b[i]=i;
					//obj->IdEvgArray[obj->ArrayCount][i]=obj->IdEvg[i];
				}

				obj->IdEvg[0]=0;obj->IdEvg[1]=2;obj->IdEvg[2]=4;obj->IdEvg[3]=6;obj->IdEvg[4]=8;obj->IdEvg[5]=10;
				obj->IdEvg[6]=1;obj->IdEvg[7]=3;obj->IdEvg[8]=5;obj->IdEvg[9]=7;obj->IdEvg[10]=9;obj->IdEvg[11]=11;
				BubbleSort(obj->IdEvg,b,VOL_VEC_ANGLE_SCLAR/2);

				InitPosAngleCount_Evg = (b[0] + b[1] + b[2] + b[3] + b[4])/5;
				InitPosAngleCount_Min = MATH_UINT16_T_ABS(b[0],InitPosAngleCount_Evg);
				InitPosAngleCount_Mid = MATH_UINT16_T_ABS(b[1],InitPosAngleCount_Evg);
				InitPosAngleCount_Max = MATH_UINT16_T_ABS(b[2],InitPosAngleCount_Evg);
				if( InitPosAngleCount_Min < InitPosAngleCount_Mid && InitPosAngleCount_Min < InitPosAngleCount_Max )
				{
					obj->InitPosAngleCount = b[0];
				}
				else if( InitPosAngleCount_Mid < InitPosAngleCount_Min && InitPosAngleCount_Mid < InitPosAngleCount_Max )
				{
					obj->InitPosAngleCount = b[1];
				}
				else if( InitPosAngleCount_Max < InitPosAngleCount_Min && InitPosAngleCount_Max < InitPosAngleCount_Mid )
				{
					obj->InitPosAngleCount = b[2];
				}
				
				obj->InitPosAngle = obj->InitPosAngleCount*obj->SubdivideAngle + obj->VolVecAngleOffset;
				obj->VolVecAngleOffset = obj->InitPosAngle - obj->SubdivideAngle - obj->SubdivideAngle;
				if( obj->VolVecAngleOffset > (1.0f) )
					obj->VolVecAngleOffset -= (1.0f);
				else if( obj->VolVecAngleOffset < (-1.0f) )
					obj->VolVecAngleOffset += (1.0f);
				obj->SubdivideAngle = (obj->SubdivideAngle * (0.333333f));//(0.5f) Test:(1.0f)
				if( obj->SubdivideAngle < (0.01f) )// 0.005f Test: 0.01f
				{
					obj->InitPosDetFlag = false;
				}
				else
				{
					obj->IdEvgCount = 0;
					for(i=0;i<VOL_VEC_ANGLE_SCLAR/2;i++)
					{
						obj->IdEvg[i] = (0.0f);
					}
				}
//				obj->IdEvgTmp = obj->IdEvg[obj->IdSampAngleCount];
//				obj->IdEvg[obj->IdSampAngleCount] = (0.0f);
//				obj->IdEvgCount = 0;
			}
		}
	}

	return;
}


void INITPOSDET_VolVecAngleRun(INITPOSDET_Handle handle)
{
	INITPOSDET_Obj *obj = (INITPOSDET_Obj *)handle;
	uint16_t count_temp = 0;

	if(obj->InitPosDetFlag)
	{
		if( obj->VolVecAngleIntervalTime >= VOL_VEC_ANGLE_INTERVAL_TIME_SCLAR )
		{
			if( obj->VolVecAngleCount < VOL_VEC_ANGLE_SCLAR )
			{
				count_temp = obj->VolVecAngleCount / 2;
				obj->VolVecAngle = count_temp * obj->SubdivideAngle + obj->VolVecAngleOffset;
				obj->IdSampAngle = obj->VolVecAngle;
				obj->IdSampAngleFlag = false;
				obj->IdSampAngleCount = count_temp;
				if( obj->VolVecAngleCount %2 == 1 )
				{
					obj->VolVecAngle += (0.5f);
					obj->VolVecAngleIntervalTime = 1;
					obj->IdSampAngleFlag = true;
				}
				if( obj->VolVecAngle > (1.0f) )
					obj->VolVecAngle -= (1.0f);
				else if( obj->VolVecAngle < (-1.0f) )
					obj->VolVecAngle += (1.0f);

				obj->VolVecAngleCount++;

				obj->VolVecAngleFlag = true;

				if( obj->VolVecAngleCount >= VOL_VEC_ANGLE_SCLAR )
				{
					obj->VolVecAngleCount=0;
				}
			}
		}
		else
		{
			obj->VolVecAngleIntervalTime++;
			obj->VolVecAngleFlag = false;
			obj->IdSampAngleFlag = false;
		}
	}
	else
	{
		obj->VolVecAngleFlag = false;
		obj->IdSampAngleFlag = false;
	}
	return;
}

