#include "modeldata.h"

float Data[50];
void setData(float dataNew,int index)
{
    Data[index]=dataNew;
}
float getData(int index)
{
    return Data[index];
}
