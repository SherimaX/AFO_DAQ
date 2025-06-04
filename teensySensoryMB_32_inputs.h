#ifndef CUSTOM_PDO_NAME_H
#define CUSTOM_PDO_NAME_H

//-------------------------------------------------------------------//
//                                                                   //
//     This file has been created by the Easy Configurator tool      //
//                                                                   //
//     Easy Configurator project teensySensoryMB.prj
//                                                                   //
//-------------------------------------------------------------------//


#define CUST_BYTE_NUM_OUT	0
#define CUST_BYTE_NUM_IN	128
#define TOT_BYTE_NUM_ROUND_OUT	0
#define TOT_BYTE_NUM_ROUND_IN	128


typedef union												//---- output buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_OUT];
	struct
	{
	}Cust;
} PROCBUFFER_OUT;


typedef union												//---- input buffer ----
{
	uint8_t  Byte [TOT_BYTE_NUM_ROUND_IN];
	struct
	{
		float       Force1;
		float       Force2;
		float       Force3;
		float       Force4;
		float       Force5;
		float       Force6;
		float       Force7;
		float       Force8;
		float       Force9;
		float       Force10;
		float       Force11;
		float       Force12;
		float       Force13;
		float       Force14;
		float       Force15;
		float       Force16;
		float       Force17;
		float       Force18;
		float       Force19;
		float       Force20;
		float       Force21;
		float       Force22;
		float       Force23;
		float       Force24;
		float       Force25;
		float       Force26;
		float       Force27;
		float       Force28;
		float       Force29;
		float       Force30;
		float       Force31;
		float       Force32;
	}Cust;
} PROCBUFFER_IN;

#endif