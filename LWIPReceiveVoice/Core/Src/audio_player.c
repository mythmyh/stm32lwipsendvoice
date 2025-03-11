/*
 * audio_player.c
 *
 *  Created on: Jun 9, 2020
 *      Author: admin
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tcpServerRAW.h"
#include "audio_player.h"
#include "pbuf.h"
int wav_end=0;
#define	BUFFER_SIZE					485
#define	WM8978_ADDRESS				0x1A
#define	WM8978_WIRTE_ADDRESS		(WM8978_ADDRESS << 1 | 0)
extern I2C_HandleTypeDef hi2c1;
extern I2S_HandleTypeDef hi2s2;
uint8_t audioname[30];
int wav_index=0;
uint16_t I2S_Buf0[BUFFER_SIZE] = { 0 };
uint16_t I2S_Buf1[BUFFER_SIZE] = { 0 };
uint16_t temp[BUFFER_SIZE]={0};
uint8_t *Delta = NULL;
extern int direction;
int i;
int wav_direction=1;
extern uint8_t wavsram[97000];

static void DMAEx_XferCpltCallback(struct __DMA_HandleTypeDef *hdma);
static void DMAEx_XferM1CpltCallback(struct __DMA_HandleTypeDef *hdma);
static void DMAEx_XferErrorCallback(struct __DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_I2S_Transmit_DMAEx(I2S_HandleTypeDef *hi2s,
		uint16_t *FirstBuffer, uint16_t *SecondBuffer, uint16_t Size);

HAL_StatusTypeDef WM8978_Register_Wirter(uint8_t reg_addr, uint16_t data) {
	uint8_t pData[10] = { 0 };

	pData[0] = (reg_addr << 1) | ((data >> 8) & 0x01);
	pData[1] = data & 0xFF;
	return HAL_I2C_Master_Transmit(&hi2c1, WM8978_WIRTE_ADDRESS, pData, 2, 1000);
}

void WAV_FileInit(void) {
}

void WM8978_Output_Cfg(uint8_t dacen,uint8_t bpsen)
{
	uint16_t regval=0;
	if(dacen)regval|=1<<0;	//DAC脢盲鲁枚脢鹿脛脺
	if(bpsen)
	{
		regval|=1<<1;		//BYPASS脢鹿脛脺
		regval|=5<<2;		//0dB脭枚脪忙
	}
	WM8978_Register_Wirter(50,regval);//R50脡猫脰脙
	WM8978_Register_Wirter(51,regval);//R51脡猫脰脙
}
static uint16_t WM8978_REGVAL_TBL[58]=
{
	0X0000,0X0000,0X0000,0X0000,0X0050,0X0000,0X0140,0X0000,
	0X0000,0X0000,0X0000,0X00FF,0X00FF,0X0000,0X0100,0X00FF,
	0X00FF,0X0000,0X012C,0X002C,0X002C,0X002C,0X002C,0X0000,
	0X0032,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,
	0X0038,0X000B,0X0032,0X0000,0X0008,0X000C,0X0093,0X00E9,
	0X0000,0X0000,0X0000,0X0000,0X0003,0X0010,0X0010,0X0100,
	0X0100,0X0002,0X0001,0X0001,0X0039,0X0039,0X0039,0X0039,
	0X0001,0X0001
};

uint16_t WM8978_Read_Reg(uint8_t reg)
{
	return WM8978_REGVAL_TBL[reg];
};
void WM8978_AUX_Gain(uint8_t gain)
{
	uint16_t regval;
	gain&=0X07;
	regval=WM8978_Read_Reg(47);//读取R47
	regval&=~(7<<0);			//清除原来的设置
	WM8978_Register_Wirter(47,regval|gain<<0);//设置R47
	regval=WM8978_Read_Reg(48);	//读取R48
	regval&=~(7<<0);			//清除原来的设置
	WM8978_Register_Wirter(48,regval|gain<<0);//设置R48
};


uint32_t WAV_FileRead(uint8_t *buf, uint32_t size)
{
		memcpy(buf, wavsram+(wav_index*size), size);
		wav_index+=1;
		if(wav_end){
			return 0;
		}
		if(wav_index>=10)wav_index=0;
		return 1;
}



void WM8978_LINEIN_Gain(uint8_t gain)
{
	uint16_t regval;
	gain&=0X07;
	regval=WM8978_Read_Reg(47);	//露脕脠隆R47
	regval&=~(7<<4);			//脟氓鲁媒脭颅脌麓碌脛脡猫脰脙
	WM8978_Register_Wirter(47,regval|gain<<4);//脡猫脰脙R47
	regval=WM8978_Read_Reg(48);	//露脕脠隆R48
	regval&=~(7<<4);			//脟氓鲁媒脭颅脌麓碌脛脡猫脰脙
	WM8978_Register_Wirter(48,regval|gain<<4);//脡猫脰脙R48
}

void WM8978_ADDA_Cfg(uint8_t dacen,uint8_t adcen)
{
	uint16_t regval;
	regval=WM8978_Read_Reg(3);	//读取R3
	if(dacen)regval|=3<<0;		//R3最低2个位设置为1,开启DACR&DACL
	else regval&=~(3<<0);		//R3最低2个位清零,关闭DACR&DACL.
	WM8978_Register_Wirter(3,regval);	//设置R3
	regval=WM8978_Read_Reg(2);	//读取R2
	if(adcen)regval|=3<<0;		//R2最低2个位设置为1,开启ADCR&ADCL
	else regval&=~(3<<0);		//R2最低2个位清零,关闭ADCR&ADCL.
	WM8978_Register_Wirter(2,regval);	//设置R2
}
void WM8978_Input_Cfg(uint8_t micen,uint8_t lineinen,uint8_t auxen)
{
	uint16_t regval;
	regval=WM8978_Read_Reg(2);	//读取R2
	if(micen)regval|=3<<2;		//开启INPPGAENR,INPPGAENL(MIC的PGA放大)
	else regval&=~(3<<2);		//关闭INPPGAENR,INPPGAENL.
 	WM8978_Register_Wirter(2,regval);	//设置R2

	regval=WM8978_Read_Reg(44);	//读取R44
	if(micen)regval|=3<<4|3<<0;	//开启LIN2INPPGA,LIP2INPGA,RIN2INPPGA,RIP2INPGA.
	else regval&=~(3<<4|3<<0);	//关闭LIN2INPPGA,LIP2INPGA,RIN2INPPGA,RIP2INPGA.
	WM8978_Register_Wirter(44,regval);//设置R44

	if(lineinen)WM8978_LINEIN_Gain(5);//LINE IN 0dB增益
	else WM8978_LINEIN_Gain(0);		//关闭LINE IN
	if(auxen)WM8978_AUX_Gain(7);//AUX 6dB增益
	else WM8978_AUX_Gain(0);	//关闭AUX输入
}

HAL_StatusTypeDef HAL_I2S_Transmit_DMAEx(I2S_HandleTypeDef *hi2s,
		uint16_t *FirstBuffer, uint16_t *SecondBuffer, uint16_t Size) {
	printf("hello dmaex\n");
	uint32_t tmpreg_cfgr;

	if ((FirstBuffer == NULL) || (SecondBuffer == NULL) || (Size == 0U)) {
		return HAL_ERROR;
	}

	/* Process Locked */
	__HAL_LOCK(hi2s);

	if (hi2s->State != HAL_I2S_STATE_READY) {
		__HAL_UNLOCK(hi2s);
		return HAL_BUSY;
	}

	/* Set state and reset error code */
	hi2s->State = HAL_I2S_STATE_BUSY_TX;
	hi2s->ErrorCode = HAL_I2S_ERROR_NONE;
	hi2s->pTxBuffPtr = FirstBuffer;

	tmpreg_cfgr = hi2s->Instance->I2SCFGR
			& (SPI_I2SCFGR_DATLEN | SPI_I2SCFGR_CHLEN);

	if ((tmpreg_cfgr == I2S_DATAFORMAT_24B)
			|| (tmpreg_cfgr == I2S_DATAFORMAT_32B)) {
		hi2s->TxXferSize = (Size << 1U);
		hi2s->TxXferCount = (Size << 1U);
	} else {
		hi2s->TxXferSize = Size;
		hi2s->TxXferCount = Size;
	}

	/* Set the I2S Tx DMA Half transfer complete callback */
	hi2s->hdmatx->XferHalfCpltCallback = NULL;
	hi2s->hdmatx->XferM1HalfCpltCallback = NULL;

	/* Set the I2S Tx DMA transfer complete callback */
	hi2s->hdmatx->XferCpltCallback = DMAEx_XferCpltCallback;
	hi2s->hdmatx->XferM1CpltCallback = DMAEx_XferM1CpltCallback;

	/* Set the DMA error callback */
	hi2s->hdmatx->XferErrorCallback = DMAEx_XferErrorCallback;

	/* Set the DMA abort callback */
	hi2s->hdmatx->XferAbortCallback = NULL;

	/* Enable the Tx DMA Stream/Channel */
	if (HAL_OK
			!= HAL_DMAEx_MultiBufferStart_IT(hi2s->hdmatx,
					(uint32_t) FirstBuffer, (uint32_t) &hi2s->Instance->DR,
					(uint32_t) SecondBuffer, hi2s->TxXferSize)) {
		/* Update SPI error code */
		SET_BIT(hi2s->ErrorCode, HAL_I2S_ERROR_DMA);
		hi2s->State = HAL_I2S_STATE_READY;

		__HAL_UNLOCK(hi2s);
		return HAL_ERROR;
	}

	/* Check if the I2S is already enabled */
	if (HAL_IS_BIT_CLR(hi2s->Instance->I2SCFGR, SPI_I2SCFGR_I2SE)) {
		/* Enable I2S peripheral */
		__HAL_I2S_ENABLE(hi2s);
	}

	/* Check if the I2S Tx request is already enabled */
	if (HAL_IS_BIT_CLR(hi2s->Instance->CR2, SPI_CR2_TXDMAEN)) {
		/* Enable Tx DMA Request */
		SET_BIT(hi2s->Instance->CR2, SPI_CR2_TXDMAEN);
	}

	__HAL_UNLOCK(hi2s);
	return HAL_OK;
}

static void DMAEx_XferCpltCallback(struct __DMA_HandleTypeDef *hdma)
{

	if(DMA1_Stream4->CR&(1<<19)){

	if (WAV_FileRead((uint8_t*) I2S_Buf0, sizeof(I2S_Buf0)) == 0)
	{
		Audio_Player_Stop();
	}
	//printf("cplt 111\n");

	}

}

static void DMAEx_XferM1CpltCallback(struct __DMA_HandleTypeDef *hdma)
{
	if (WAV_FileRead((uint8_t*) I2S_Buf1, sizeof(I2S_Buf1)) == 0)
	{
		Audio_Player_Stop();
	}
	//printf("m1 222\n");


}

static void DMAEx_XferErrorCallback(struct __DMA_HandleTypeDef *hdma) {

}

const uint16_t I2S_PSC_TBL[][5]=
{
	{800 ,256,5,12,1},		//8Khz������
	{1102,429,4,19,0},		//11.025Khz������
	{1600,213,2,13,0},		//16Khz������
	{2205,429,4, 9,1},		//22.05Khz������
	{3200,213,2, 6,1},		//32Khz������
	{4410,271,2, 6,0},		//44.1Khz������
	{4800,258,3, 3,1},		//48Khz������
	{8820,316,2, 3,1},		//88.2Khz������
	{9600,344,2, 3,1},  	//96Khz������
	{17640,361,2,2,0},  	//176.4Khz������
	{19200,393,2,2,0},  	//192Khz������
};

//����I2S��DMA����,HAL��û���ṩ�˺���
//���������Ҫ�Լ������Ĵ�����дһ��
void I2S_DMA_Enable(void)
{
    uint32_t tempreg=0;
    tempreg=SPI2->CR2;    	//�ȶ�����ǰ������
	tempreg|=1<<1;			//ʹ��DMA
	SPI2->CR2=tempreg;		//д��CR1�Ĵ�����
}

//����SAIA�Ĳ�����(@MCKEN)
//samplerate:������,��λ:Hz
//����ֵ:0,���óɹ�;1,�޷�����.
uint8_t I2S2_SampleRate_Set(uint32_t samplerate)
{
    uint8_t i=0;
	uint32_t tempreg=0;
    RCC_PeriphCLKInitTypeDef RCCI2S2_ClkInitSture;

	for(i=0;i<(sizeof(I2S_PSC_TBL)/10);i++)//�����Ĳ������Ƿ����֧��
	{
		if((samplerate/10)==I2S_PSC_TBL[i][0])break;
	}
    if(i==(sizeof(I2S_PSC_TBL)/10))return 1;//�ѱ���Ҳ�Ҳ���

    RCCI2S2_ClkInitSture.PeriphClockSelection=RCC_PERIPHCLK_I2S;	//����ʱ��Դѡ��
    RCCI2S2_ClkInitSture.PLLI2S.PLLI2SN=(uint32_t)I2S_PSC_TBL[i][1];    	//����PLLI2SN
    RCCI2S2_ClkInitSture.PLLI2S.PLLI2SR=(uint32_t)I2S_PSC_TBL[i][2];    	//����PLLI2SR
    HAL_RCCEx_PeriphCLKConfig(&RCCI2S2_ClkInitSture);             	//����ʱ��

	RCC->CR|=1<<26;					//����I2Sʱ��
	while((RCC->CR&1<<27)==0);		//�ȴ�I2Sʱ�ӿ����ɹ�.
	tempreg=I2S_PSC_TBL[i][3]<<0;	//����I2SDIV
	tempreg|=I2S_PSC_TBL[i][4]<<8;	//����ODDλ
	tempreg|=1<<9;					//ʹ��MCKOEλ,���MCK
	SPI2->I2SPR=tempreg;			//����I2SPR�Ĵ���
	return 0;
}


void Audio_Player_Init(void) {

	WM8978_Register_Wirter(0, 0);
	WM8978_Register_Wirter(1, 0x0F);
	WM8978_Register_Wirter(2, 0x180);	// ģ��Ŵ���ʹ�ܣ�?? ʹ��������뻺����??
	WM8978_Register_Wirter(3, 0x7F);
	WM8978_Register_Wirter(4, 0x10);
	WM8978_Register_Wirter(6, 0);
	WM8978_Register_Wirter(10, 0x08);
	WM8978_Register_Wirter(43, 0x10);
	WM8978_Register_Wirter(52,40);		// 设置LOUT2左声道音�????
	WM8978_Register_Wirter(53,40|(1<<8));
	WM8978_Register_Wirter(54, 40);
	WM8978_Register_Wirter(55, 40 | (1 << 8));


}

void Audio_Set_Volume(int num) {
	WM8978_Register_Wirter(0, 0);
	WM8978_Register_Wirter(1, 0x0F);
	WM8978_Register_Wirter(2, 0x180);	// ģ��Ŵ���ʹ�ܣ�?? ʹ��������뻺����??
	WM8978_Register_Wirter(3, 0x7F);
	WM8978_Register_Wirter(4, 0x10);
	WM8978_Register_Wirter(6, 0);
	WM8978_Register_Wirter(10, 0x08);
	WM8978_Register_Wirter(43, 0x10);
	WM8978_Register_Wirter(52, num);		// 设置LOUT2左声道音�????
	WM8978_Register_Wirter(53, num | (1 << 8));
	WM8978_Register_Wirter(54, num);
	WM8978_Register_Wirter(55, num | (1 << 8));
}

void Audio_Player_Start() {

	wav_index=0;
	HAL_I2S_Transmit_DMAEx(&hi2s2, I2S_Buf0, I2S_Buf1, BUFFER_SIZE);
}

void Audio_Player_Pause(void) {
	HAL_I2S_DMAPause(&hi2s2);
}

void Audio_Player_Resume(void) {
	HAL_I2S_DMAResume(&hi2s2);
}

void Audio_Player_Stop(void) {
	printf("stop\n");
	HAL_I2S_DMAStop(&hi2s2);
}

