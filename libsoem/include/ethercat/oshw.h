/******************************************************************************
 *                *          ***                    ***
 *              ***          ***                    ***
 * ***  ****  **********     ***        *****       ***  ****          *****
 * *********  **********     ***      *********     ************     *********
 * ****         ***          ***              ***   ***       ****   ***
 * ***          ***  ******  ***      ***********   ***        ****   *****
 * ***          ***  ******  ***    *************   ***        ****      *****
 * ***          ****         ****   ***       ***   ***       ****          ***
 * ***           *******      ***** **************  *************    *********
 * ***             *****        ***   *******   **  **  ******         *****
 *                           t h e  r e a l t i m e  t a r g e t  e x p e r t s
 *
 * http://www.rt-labs.com
 * Copyright (C) 2009. rt-labs AB, Sweden. All rights reserved.
 *------------------------------------------------------------------------------
 * $Id: oshw.h 452 2013-02-26 21:02:58Z smf.arthur $
 *------------------------------------------------------------------------------
 */

/** \file 
 * \brief
 * Headerfile for ethercatbase.c 
 */

#ifndef ETHERCAT_OSHW_H
#define ETHERCAT_OSHW_H

#include <ethercat/type.h>
#include <ethercat/nicdrv.h>
#include <ethercat/main.h>

uint16 oshw_htons(uint16 hostshort);
uint16 oshw_ntohs(uint16 networkshort);
ec_adaptert * oshw_find_adapters(void);
void oshw_free_adapters(ec_adaptert * adapter);

#endif /* ETHERCAT_OSHW_H */

