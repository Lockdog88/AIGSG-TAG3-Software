/* Includes ----------------------------------------------------------------- */
#include "eeprom.h"
#include "qm_i2c.h"
#include "clk.h"


int _debug = 0;
int _i2c_address = I2C_ADDR_24LCXXX;

int M24C64_byte_write( int mem_addr, uint8_t data )
{
    int res;
    uint8_t buf[3];
    qm_i2c_status_t status;

    buf[0] = mem_addr>>8;        // Write Address High byte set
    buf[1] = 0xFF & mem_addr;           // Write Address Low  byte set
    buf[2] = data;

    res = qm_i2c_master_write(QM_I2C_0, _i2c_address, buf, sizeof(buf), true,
			 &status);

    if(_debug)
    {
        if(res==0)
        {
        	QM_PRINTF("Success! Byte Data Write. \r\n");
        }
        else
        {
        	QM_PRINTF("Failed! Byte Data Write %d.\r\n", res);
        }
    }

    clk_sys_udelay(5000);
 
    return res;
}

int M24C64_nbyte_write( int mem_addr, void *data, int size )
{
    int  i;
    int  res;
    char buf[3];
    char *p;
    qm_i2c_status_t status;

    p = (char *)data;
    res = -1;
    for ( i = 0; i < size; i++ )
    {
        buf[0] = (0xff00 & mem_addr)>>8;        // Read Address High byte set
        buf[1] = (0x00ff & mem_addr);           // Read Address Low  byte set
        buf[2] = *p++;

        res = qm_i2c_master_write(QM_I2C_0, _i2c_address, (uint8_t*)buf, sizeof(buf), false,
   			 &status);
        if(_debug)
        {
            if(res==0)
            {
            	QM_PRINTF("Success! N-Byte Data Write. \r\n");
            }
            else
            {
            	QM_PRINTF("Failed! N-Byte Data Write %d.\r\n", res);
            }
        }

        if(res!=0)
        {
            return res;
        }

        if( ++mem_addr >= MAXADR_24LCXXX )      // Address counter +1
        {
            return -1;                          // Address range over
        }
    }
    clk_sys_udelay(5000);

    return res;
}

int M24C64_page_write( int mem_addr, char *data )
{
    int i;
    int res;
    char buf[PAGE_SIZE_24LCXXX+2];
    qm_i2c_status_t status;

    buf[0] = (0xff00 & mem_addr)>>8;        // Write Address High byte set
    buf[1] = (0x00ff & mem_addr);           // Write Address Low  byte set

    for (i=0; i<PAGE_SIZE_24LCXXX; i++)
    {
        buf[i+2] = data[i];
    }
    res = qm_i2c_master_write(QM_I2C_0, _i2c_address, (uint8_t*)buf, sizeof(buf), false,
  			 &status);
    if(_debug)
    {
        if(res==0)
        {
        	QM_PRINTF("Success! Page Data Write. \r\n");
        }
        else
        {
        	QM_PRINTF("Failed! Page Data Write %d.\r\n", res);
        }
    }

    clk_sys_udelay(5000);

    return res;
}


int M24C64_nbyte_read( int mem_addr, unsigned char *const data, int size )
{
    int res;
    unsigned char addr[2];
    qm_i2c_status_t status;

    addr[0] = (mem_addr>>8);        // Read Address High byte set
    addr[1] = (0xff & mem_addr);           // Read Address Low  byte set
    res = qm_i2c_master_write(QM_I2C_0, _i2c_address, addr, sizeof(addr), false, &status);

    if(_debug)
    {
        if(res==0)
        {
            QM_PRINTF("Success! nbyte read address send. \r\n");
        }
        else
        {
            QM_PRINTF("Failed! nbyte read address send %d.\r\n", res);
        }
    }
 


    res = qm_i2c_master_read(QM_I2C_0, _i2c_address, data, sizeof(data), true, &status);
    if(_debug)
    {
        if(res==0)
        {
        	QM_PRINTF("Success! nbyte read address send. \r\n");
        }
        else
        {
        	QM_PRINTF("Failed! nbyte read address send %d.\r\n", res);
        }
    }

    clk_sys_udelay(5000);
 
    return res;
}
