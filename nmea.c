#include "nmea.h"

uint8_t nmea_header_detect(nmea_header_t type);
char** str_split(char* a_str, const char a_delim);

/*
 * ret=0: Data ready
 * ret=1: Timeout
 * ret=2: Invalid header
 */
gnss_gprmc_t read_gprmc(void)
{
	//uint8_t * data = (uint8_t*) malloc(85);
	uint8_t data[85] = {};
	gnss_gprmc_t nmea;
	qm_uart_status_t uart_status = 0;
	char** tokens;
	//memset(data,0,85);
	nmea.ret = 1;
	if (nmea_header_detect(NMEA_GPRMC) == 0) {
		for (uint8_t i=0; i<85; i++){
			qm_uart_get_status(QM_UART_0, &uart_status);
			if (uart_status & QM_UART_RX_BUSY) qm_uart_read(QM_UART_0, &data[i], &uart_status); else i--;
			if((data[i]==0x0A) || (data[i]==0x0D)) {
				data[i]='#';
				nmea.ret = 0;
				break;
			}
		}
		strcpy((char*)nmea.header, "$GPRMC");
	    if (nmea.ret == 0){
	    	tokens = str_split((char*)data, ',');
	    	if (tokens)
	    	    {
	    	        int i;
	    	        for (i = 0; *(tokens + i); i++)
	    	        {
	    	        	if (i==0) { strcpy((char*)nmea.time,*(tokens + i)); free(*(tokens + i));}
	    	        	if (i==1) { strcpy((char*)&nmea.warning,*(tokens + i)); free(*(tokens + i));}
	    	        	if ((i==5) && (nmea.warning=='V')){nmea.ret=3;strcpy((char*)nmea.checksum,*(tokens + i)); free(*(tokens + i));}
	    	        	if (nmea.warning=='A')
	    	        	{
	    	        		if(i==2) { strcpy((char*)nmea.latitude,*(tokens + i)); free(*(tokens + i));}
	    	        		if(i==3) { strcpy((char*)&nmea.ns,*(tokens + i)); free(*(tokens + i));}
	    	        		if(i==4) { strcpy((char*)nmea.longtitude,*(tokens + i)); free(*(tokens + i));}
	    	        		if(i==5) { strcpy((char*)&nmea.ew,*(tokens + i)); free(*(tokens + i));}
	    	        		nmea.ret=0;
	    	        	}
	    	            free(*(tokens + i));
	    	        }
	    	        free(tokens);
	    	    }
	    }
	} else nmea.ret = 2;
	//memset(data,0,85);
	//free(data);
	return nmea;
}

/*
 * ret=0: Valid header
 * ret=1: Empty header (timeout)
 * ret=2: Invalid header
 */
uint8_t nmea_header_detect(nmea_header_t type)
{
	//uint8_t * data = (uint8_t*) malloc(6);
	uint8_t data[6] = {};
	uint8_t timeout = 0;
	uint8_t ret = 0;

	qm_uart_status_t uart_status = 0;
	//memset(data,0,6);
	data[0] = '0';
	while(data[0]!='$') {
		qm_uart_get_status(QM_UART_0, &uart_status);
		if (uart_status & QM_UART_RX_BUSY) {
			qm_uart_read(QM_UART_0, &data[0], &uart_status);
			if (timeout>100) { ret = 1; debug_str("HEADER TIMEOUT"); break; } else timeout++;
		}
	}
	timeout=0;
	data[0] = '0';
	while(data[0]!='$') {
		qm_uart_get_status(QM_UART_0, &uart_status);
		if (uart_status & QM_UART_RX_BUSY) {
			qm_uart_read(QM_UART_0, &data[0], &uart_status);
			if (timeout>100) { ret = 1; debug_str("HEADER TIMEOUT"); break; } else timeout++;
		}
	}
	timeout=0;
	if (data[0]=='$') {
		for (uint8_t i = 1; i<6; i++){
			qm_uart_get_status(QM_UART_0, &uart_status);
			if (uart_status & QM_UART_RX_BUSY) {
				qm_uart_read(QM_UART_0, &data[i], &uart_status);
			} else i--;
		}

		if (type==NMEA_GPRMC) { if(strncmp((char*)data, "$GPRMC", 6)==0) ret=0; else ret=1; }
	} else {
		ret=2;
	}
	//memset(data,0,6);
	//free(data);
	return ret;
}


char** str_split(char* a_str, const char a_delim)
{
    char** result    = 0;
    size_t count     = 0;
    char* tmp        = a_str;
    char* last_comma = 0;
    char delim[2];
    delim[0] = a_delim;
    delim[1] = 0;

    /* Count how many elements will be extracted. */
    while (*tmp)
    {
        if (a_delim == *tmp)
        {
            count++;
            last_comma = tmp;
        }
        tmp++;
    }

    /* Add space for trailing token. */
    count += last_comma < (a_str + strlen(a_str) - 1);

    /* Add space for terminating null string so caller
       knows where the list of returned strings ends. */
    count++;

    result = malloc(sizeof(char*) * count);

    if (result)
    {
        size_t idx  = 0;
        char* token = strtok(a_str, delim);

        while (token)
        {
            //QM_ASSERT(idx < count);
            *(result + idx++) = strdup(token);
            token = strtok(0, delim);
        }
        //QM_ASSERT(idx == count - 1);
        *(result + idx) = 0;
    }

    return result;
}
