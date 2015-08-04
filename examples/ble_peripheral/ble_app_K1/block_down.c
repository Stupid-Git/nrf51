
#include "stdint.h"

#define BLK_DN_COUNT 128

uint8_t m_blkDn_buf[ 16 * BLK_DN_COUNT ]; // 2048 @ BLK_DN_COUNT = 128
uint8_t m_blkDn_chk[  1 * BLK_DN_COUNT ]; //  128 @ BLK_DN_COUNT = 128
uint16_t m_blkDn_len;
uint16_t m_blkDn_blkCnt;
uint16_t m_blkDn_rxBlkCnt;

int32_t blk_dn_start( uint8_t *pkt )
{
    uint8_t  j;
    uint16_t i;

    m_blkDn_rxBlkCnt = 0;
    m_blkDn_blkCnt = 0;    
    
    m_blkDn_len = pkt[2] | (pkt[3]<<8);
    if( m_blkDn_len == 0)
        return(1);
    
    m_blkDn_blkCnt = ((m_blkDn_len - 1) / 16) + 1;
    
    
    for(i=0 ; i < m_blkDn_blkCnt; i++)
    {
        m_blkDn_chk[i] = 0x00;
        for(j=0 ; j < 16; j++)
        {
            m_blkDn_buf[i*16 + j] = 0x00;
        }
    }
    
    return(0);
}


int32_t blk_dn_add( uint8_t *pkt, uint16_t len )
{
    uint8_t  position;
    uint8_t  j;
    uint16_t i;
    
    if(len != 20)
        return(1);
    
    position = pkt[0];
    if( position >= BLK_DN_COUNT)
        return(1);
    
    m_blkDn_rxBlkCnt++;
    m_blkDn_chk[position] += 1;
    for(j=0 ; j < 16; j++)
    {
        m_blkDn_buf[position*16 + j] = pkt[4 + j];
    }
    return(0);
}
int32_t blk_dn_chk()
{
    uint8_t  j;
    uint16_t i;
    uint16_t missing_blk_cnt;
    uint16_t cs_pkt;
    uint16_t cs_now;
 
    if(m_blkDn_blkCnt == 0)
        return(1);

    if(m_blkDn_rxBlkCnt < m_blkDn_blkCnt)
        return(1);
    
    missing_blk_cnt = 0;
    for(i=0 ; i < m_blkDn_blkCnt; i++)
    {
        if(m_blkDn_chk[i] == 0x00)
            missing_blk_cnt++;
    }
    if(missing_blk_cnt>0)
        return(2);

    cs_pkt = m_blkDn_buf[m_blkDn_len - 2] | (m_blkDn_buf[m_blkDn_len - 1]<<8);
    cs_now = 0;
    for(i=0 ; i < m_blkDn_len - 2; i++)
    {
        cs_now += m_blkDn_buf[i];
    }
    if( cs_now != cs_pkt)
        return(3);
    
    return(0);
}

