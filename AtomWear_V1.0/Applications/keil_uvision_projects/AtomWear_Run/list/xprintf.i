#line 1 "..\\..\\..\\Software\\Source\\xprintf\\xprintf.c"











 

#line 1 "..\\..\\..\\Software\\Include\\xprintf.h"
 
 
 













extern void (*xfunc_out)(unsigned char);
void xputc (char c);
void xputs (const char* str);
void xfputs (void (*func)(unsigned char), const char* str);
void xprintf (const char* fmt, ...);
void xsprintf (char* buff, const char* fmt, ...);
void xfprintf (void (*func)(unsigned char), const char*	fmt, ...);
void put_dump (const void* buff, unsigned long addr, int len, int width);





#line 37 "..\\..\\..\\Software\\Include\\xprintf.h"

#line 15 "..\\..\\..\\Software\\Source\\xprintf\\xprintf.c"
#line 1 "..\\..\\..\\Software\\Include\\simple_uart.h"
 









 




 

#line 1 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdbool.h"
 






 





#line 25 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdbool.h"



#line 19 "..\\..\\..\\Software\\Include\\simple_uart.h"
#line 1 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"
 
 





 










#line 26 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"







 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 197 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"

     







     










     











#line 261 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdint.h"



 



#line 20 "..\\..\\..\\Software\\Include\\simple_uart.h"









 




 
uint8_t simple_uart_get(void);






 
_Bool simple_uart_get_with_timeout(int32_t timeout_ms, uint8_t *rx_data);




 
void simple_uart_put(uint8_t cr);





 
void simple_uart_putstring(const uint8_t *str);







 
void simple_uart_config(uint8_t rts_pin_number, uint8_t txd_pin_number, uint8_t cts_pin_number, uint8_t rxd_pin_number, _Bool hwfc);



 

 
#line 16 "..\\..\\..\\Software\\Source\\xprintf\\xprintf.c"


#line 1 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdarg.h"
 
 
 





 










#line 27 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdarg.h"








 

 
 
  typedef struct __va_list { void *__ap; } va_list;

   






 


   










 


   















 




   

 


   




 



   





 






#line 119 "C:\\Keil\\ARM\\ARMCC\\bin\\..\\include\\stdarg.h"











 

#line 20 "..\\..\\..\\Software\\Source\\xprintf\\xprintf.c"

void (*xfunc_out)(unsigned char);	 
static char *outptr;

 
 
 

void xputc (char c)
{
	if (0 && c == '\n') xputc('\r');		 

	if (outptr) {
		*outptr++ = (unsigned char)c;
		return;
	}

        
        simple_uart_put( c);
	
}



 
 
 

void xputs (					 
	const char* str				 
)
{
	while (*str)
		xputc(*str++);
}


void xfputs (					 
	void(*func)(unsigned char),	 
	const char*	str				 
)
{
	void (*pf)(unsigned char);


	pf = xfunc_out;		 
	xfunc_out = func;	 
	while (*str)		 
		xputc(*str++);
	xfunc_out = pf;		 
}



 
 
 












 

static
void xvprintf (
	const char*	fmt,	 
	va_list arp			 
)
{
	unsigned int r, i, j, w, f;
	unsigned long v;
	char s[16], c, d, *p;


	for (;;) {
		c = *fmt++;					 
		if (!c) break;				 
		if (c != '%') {				 
			xputc(c); continue;
		}
		f = 0;
		c = *fmt++;					 
		if (c == '0') {				 
			f = 1; c = *fmt++;
		} else {
			if (c == '-') {			 
				f = 2; c = *fmt++;
			}
		}
		for (w = 0; c >= '0' && c <= '9'; c = *fmt++)	 
			w = w * 10 + c - '0';
		if (c == 'l' || c == 'L') {	 
			f |= 4; c = *fmt++;
		}
		if (!c) break;				 
		d = c;
		if (d >= 'a') d -= 0x20;
		switch (d) {				 
		case 'S' :					 
			p = __va_arg(arp, char*);
			for (j = 0; p[j]; j++) ;
			while (!(f & 2) && j++ < w) xputc(' ');
			xputs(p);
			while (j++ < w) xputc(' ');
			continue;
		case 'C' :					 
			xputc((char)__va_arg(arp, int)); continue;
		case 'B' :					 
			r = 2; break;
		case 'O' :					 
			r = 8; break;
		case 'D' :					 
		case 'U' :					 
			r = 10; break;
		case 'X' :					 
			r = 16; break;
		default:					 
			xputc(c); continue;
		}

		 
		v = (f & 4) ? __va_arg(arp, long) : ((d == 'D') ? (long)__va_arg(arp, int) : (long)__va_arg(arp, unsigned int));
		if (d == 'D' && (v & 0x80000000)) {
			v = 0 - v;
			f |= 8;
		}
		i = 0;
		do {
			d = (char)(v % r); v /= r;
			if (d > 9) d += (c == 'x') ? 0x27 : 0x07;
			s[i++] = d + '0';
		} while (v && i < sizeof(s));
		if (f & 8) s[i++] = '-';
		j = i; d = (f & 1) ? '0' : ' ';
		while (!(f & 2) && j++ < w) xputc(d);
		do xputc(s[--i]); while(i);
		while (j++ < w) xputc(' ');
	}
}


void xprintf (			 
	const char*	fmt,	 
	...					 
)
{
	va_list arp;


	__va_start(arp, fmt);
	xvprintf(fmt, arp);
	__va_end(arp);
}


void xsprintf (			 
	char* buff,			 
	const char*	fmt,	 
	...					 
)
{
	va_list arp;


	outptr = buff;		 

	__va_start(arp, fmt);
	xvprintf(fmt, arp);
	__va_end(arp);

	*outptr = 0;		 
	outptr = 0;			 
}


void xfprintf (					 
	void(*func)(unsigned char),	 
	const char*	fmt,			 
	...							 
)
{
	va_list arp;
	void (*pf)(unsigned char);


	pf = xfunc_out;		 
	xfunc_out = func;	 

	__va_start(arp, fmt);
	xvprintf(fmt, arp);
	__va_end(arp);

	xfunc_out = pf;		 
}



 
 
 

void put_dump (
	const void* buff,		 
	unsigned long addr,		 
	int len,				 
	int width				 
)
{
	int i;
	const unsigned char *bp;
	const unsigned short *sp;
	const unsigned long *lp;


	xprintf("%08lX ", addr);		 

	switch (width) {
	case sizeof(char):
		bp = buff;
		for (i = 0; i < len; i++)		 
			xprintf(" %02X", bp[i]);
		xputc(' ');
		for (i = 0; i < len; i++)		 
			xputc((bp[i] >= ' ' && bp[i] <= '~') ? bp[i] : '.');
		break;
	case sizeof(short):
		sp = buff;
		do								 
			xprintf(" %04X", *sp++);
		while (--len);
		break;
	case sizeof(long):
		lp = buff;
		do								 
			xprintf(" %08LX", *lp++);
		while (--len);
		break;
	}

	xputc('\n');
}





