/* 
 * File:   flash.h
 * Author: navin.gautam
 *
 * Created on July 31, 2015, 3:30 PM
 */

#ifndef FLASH_H
#define	FLASH_H

#ifdef	__cplusplus
extern "C" {
#endif

void FlashEraseRow( const void *address );
void FlashWriteRow( const void *address, void *data );
void FlashReadRow( const void *address, void *data );


#ifdef	__cplusplus
}
#endif

#endif	/* FLASH_H */

