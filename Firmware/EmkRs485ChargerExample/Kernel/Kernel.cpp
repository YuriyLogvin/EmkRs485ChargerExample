/*
 * Kernel.cpp
 *
 *  Created on: 6 вер. 2019 р.
 *      Author: banz
 */

#include "Kernel.h"
#include "Hal.h"
#include "ProtocolHost.h"
#include "ReceiveMetodHost.h"
#include "SendMetodHost.h"
#include "EmkStandartMetods.h"
#include "InterfaceMetodsCharger.h"
#include "cmsis_os.h"

extern "C" void tskTransmit2UartStart(void const * argument);
extern "C" void tskMainStart(void const * argument);

ProtocolHost* _ProtocolHost;
ReceiveMetodHost* _ReceiveMetodHost;
SendMetodHost* _SendMetodHost;


void BmsKernelInit()
{

	Hal::Init();

	Kernel::Init();

}

void tskMainStart(void const * argument)
{

  for(;;)
  {
		Hal::Tick();
		Kernel::Tick();
		Hal::UpdateWdt();
		osDelay(0);
  }
}

void tskTransmit2UartStart(void const * argument)
{
  for(;;)
  {
	  Kernel::TransmitState2Usart();
	  osDelay(1000);
  }
}




void Kernel::Init()
{
	_ProtocolHost = new ProtocolHost(EmkAddr::Charger);
	_ProtocolHost->DestAddr(EmkAddr::Host);

	_ReceiveMetodHost = new ReceiveMetodHost();

	_SendMetodHost = new SendMetodHost();

}

int32_t _KernelTicks = 0;

void Kernel::Tick()
{

	uint8_t b = 0;
	uint8_t* data = 0;
	uint8_t len = 0;
	for (;Hal::Rs485->Receive(&b, 1) > 0;)
	{
		data = _ProtocolHost->ReceiveData(b, len);
		if (data)
			break;
	}


	if (data != 0)
	{
		_ReceiveMetodHost->InitNewMetod(data, len);
		_ProcessDataPacket();
		return;
	}

	//Blink LED logic
	if (Hal::GetSpendTicks(_KernelTicks) < Hal::GetTicksInMilliSecond() * 10)
		return;

	Hal::LedBlue(!Hal::LedBlue());

	_KernelTicks = Hal::GetTickCount();
}

static bool _EnableCharging = 0;
static int16_t _SetVoltage = 0;
static int16_t _SetCurrent = 0;

static int16_t _HaveVoltage = 0;
static int16_t _HaveCurrent = 0;
static ChargerStates _HaveState = ChargerStates::None;

void Kernel::_ProcessDataPacket()
{

	auto mNum = _ReceiveMetodHost->GetMetodNumber();

	if (_ProtocolHost->PacketAddr() != EmkAddr::Charger)
		return;

	if ((EmkMetods)mNum == EmkMetods::Ping)
	{
		_ResponsePing();
		return;
	};

	switch ((InterfaceMetodsCharger)mNum)
	{
	case InterfaceMetodsCharger::ProcessCharging:
		{
		bool enableCharging = false;
		int16_t voltage = 0;
		int16_t current = 0;
		if (!_ReceiveMetodHost->GetArgumentBool(0, enableCharging))
			break;
		if (!_ReceiveMetodHost->GetArgumentShort(1, voltage))
			break;
		if (!_ReceiveMetodHost->GetArgumentShort(2, current))
			break;

		_EnableCharging = enableCharging;
		_SetVoltage = voltage;
		_SetCurrent = current;

		_SendMetodHost->InitNewMetod(mNum);

		_SendMetodHost->AddArgumentByte((uint8_t)_HaveState);

		_SendMetodHost->AddArgumentShort(_HaveVoltage);

		_SendMetodHost->AddArgumentShort(_HaveCurrent);

		_SendData();

		break;
		}
	default:
		break;
	};

}

void Kernel::TransmitState2Usart()
{
	while (!Hal::Usart->Redy4Send())
		 osDelay(1);
	Hal::Usart->Send("%i:%i:%i\r\n",_HaveState,_HaveVoltage,_HaveCurrent);
}

volatile int _ReceivedPings = 0;

void Kernel::_ResponsePing()
{
	_SendMetodHost->InitNewMetod((uint8_t)EmkMetods::Ping);

	_SendMetodHost->AddArgumentBool(true);

	//version
	//_SendMetodHost->AddArgumentByte(2);

	_ReceivedPings++;

	_SendData();
}

void Kernel::_SendData()
{

	_ProtocolHost->InitSendData(*_SendMetodHost);

	uint8_t buff[32];

	uint8_t pos = 0;
	for (; pos < sizeof(buff); pos++)
	{
		uint8_t b;
		if (!_ProtocolHost->SendData(b))
			break;
		buff[pos] = b;
	}

	if (pos)
	{
		HAL_Delay(1); //ћежду приемом и отправкой должно быть не менее 100мк—ек
		Hal::Rs485->Send(buff, pos);
	}

}


void Kernel::Send(uint8_t* data, uint16_t len)
{
	Hal::Rs485->Send(data, len);
}

