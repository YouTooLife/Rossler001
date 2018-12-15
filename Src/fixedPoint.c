/*
 * fixedPoint.c
 *
 *  Created on: Dec 10, 2018
 *      Author: youtoolife
 */

#include <stdint.h>

#define FI int32_t

#define FPQ 16


//#define FADD0(a,b) ((a)+(b))

FI fadd(FI a, FI b)
{
	return a+b;
}

//#define FSUB(a,b) ((a)-(b))

FI fsub(FI a, FI b)
{
	return a-b;
}

//#define FMUL(a,b,q) (((a)*(b))>>(q))


FI fmul(FI a, FI b, uint8_t q)
{
int64_t a2 = a;
return (a2*b) >> q;
}


FI fmul2(FI a, FI b, uint8_t q)
{
		FI inta = a >> q;
		FI intb = b >> q;
		FI fracta = a & (0xffffffff >> (32-q));
		FI fractb = b & (0xffffffff >> (32-q));
		FI result = (inta * intb << q) + (inta * fractb + fracta * intb) + (fracta * fractb >> q);
	    return result;
}

//#define FDIV(a,b,q) (((a)<<(q))/(b))
FI fdiv(FI a, FI b, uint8_t q)
{
int64_t a2 = a;
return  (a2 << q) / b;
}


FI fdiv2(FI a, FI b, uint8_t q)
{
		FI r = a >> q, q2 = a << q;
	    uint_fast8_t carry = 0;
	    for (uint_fast8_t i = 0; i < 32; i++)
	    {
	        if(r & 0x80000000)
	            carry = 1;
	        else
	            carry = 0;
	        r <<= 1;
	        if (q2 & 0x80000000)
	            r |= 1;
	        q2 <<= 1;
	        if (r >= b || carry)
	        {
	            q2 = q2 | 1;
	            r -= b;
	        }
	    }
	    return q2;
}


/*
#define FADDI(a,b,q) ((a)+((b)<<(q)))
#define FSUBI(a,b,q) ((a)-((b)<<(q)))
#define FMULI(a,b) ((a)*(b))
#define FDIVI(a,b) ((a)/(b))

#define FCONV(a, q1, q2) (((q2)>(q1)) ? (a)<<((q2)-(q1)) : (a)>>((q1)-(q2)))

#define FADDG(a,b,q1,q2,q3) (FCONV(a,q1,q3)+FCONV(b,q2,q3))
#define FSUBG(a,b,q1,q2,q3) (FCONV(a,q1,q3)-FCONV(b,q2,q3))
#define FMULG(a,b,q1,q2,q3) FCONV((a)*(b), (q1)+(q2), q3)
#define FDIVG(a,b,q1,q2,q3) (FCONV(a, q1, (q2)+(q3))/(b))
*/

//#define TOFIX(d, q) ((int32_t)( (d)*(double)(1<<(q)) ))

FI toFix(double d, uint8_t q)
{
	return (FI) ( d*(double)(1<<q));
}

//#define TOFLT(a, q) ( (double)(a) / (double)(1<<(q)) )

double toFlt(FI a, uint8_t q)
{
	return (double)(a) / (double)(1<<(q));
}






