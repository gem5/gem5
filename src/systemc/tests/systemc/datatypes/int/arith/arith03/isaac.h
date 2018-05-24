#ifndef __ISAAC_HPP
#define __ISAAC_HPP


/*

    C++ TEMPLATE VERSION OF Robert J. Jenkins Jr.'s
    ISAAC Random Number Generator.

    Ported from vanilla C to to template C++ class
    by Quinn Tyler Jackson on 16-23 July 1998.

        quinn@qtj.net

    The function for the expected period of this
    random number generator, according to Jenkins is:

        f(a,b) = 2**((a+b*(3+2^^a)-1)

        (where a is ALPHA and b is bitwidth)
        
    So, for a bitwidth of 32 and an ALPHA of 8,
    the expected period of ISAAC is:

        2^^(8+32*(3+2^^8)-1) = 2^^8295

    Jackson has been able to run implementations
    with an ALPHA as high as 16, or

        2^^2097263

*/


typedef unsigned int UINT32;
const UINT32 GOLDEN_RATIO = UINT32(0x9e3779b9);


template <UINT32 ALPHA = (8)>
class QTIsaac
{
  public:
   
    typedef unsigned char byte;
   
    struct randctx
    {
        randctx(void)
        {
            randrsl = new UINT32[N];
            randmem = new UINT32[N];
        }
      
        ~randctx(void)
        {
            delete [] randrsl;
            delete [] randmem;
        }
      
        UINT32 randcnt;
        UINT32* randrsl;
        UINT32* randmem;
        UINT32 randa;
        UINT32 randb;
        UINT32 randc;
	};
   
    QTIsaac(UINT32 a = 0, UINT32 b = 0, UINT32 c = 0);
    virtual ~QTIsaac(void);
   
    UINT32 rand(void);
    virtual void randinit(randctx* ctx, bool bUseSeed);
    virtual void srand(
		UINT32 a = 0, UINT32 b = 0, UINT32 c = 0, UINT32* s = NULL);
   
    enum {N = (1<<ALPHA)};
   
  protected:
   
     virtual void isaac(randctx* ctx);
   
     UINT32 ind(UINT32* mm, UINT32 x);
     void rngstep(
		UINT32 mix, UINT32& a, UINT32& b, UINT32*& mm, UINT32*& m, 
		UINT32*& m2, UINT32*& r, UINT32& x, UINT32& y);
     virtual void shuffle(
		UINT32& a, UINT32& b, UINT32& c, UINT32& d, UINT32& e, UINT32& f, 
		UINT32& g, UINT32& h);
   
  private:
    randctx m_rc;   
};


template<UINT32 ALPHA>
QTIsaac<ALPHA>::QTIsaac(UINT32 a, UINT32 b, UINT32 c) : m_rc()
{
    srand(a, b, c);
}


template<UINT32 ALPHA>
QTIsaac<ALPHA>::~QTIsaac(void)
{
    // DO NOTHING
}


template<UINT32 ALPHA>
void QTIsaac<ALPHA>::srand(UINT32 a, UINT32 b, UINT32 c, UINT32* s)
{
	for(int i = 0; i < N; i++)
	{
		m_rc.randrsl[i] = s != NULL ? s[i] : 0;
	}
   
	m_rc.randa = a;
	m_rc.randb = b;
	m_rc.randc = c;
   
	randinit(&m_rc, true);
}


template<UINT32 ALPHA>
inline UINT32 QTIsaac<ALPHA>::rand(void)
{
	return 0x7fffffff & (!m_rc.randcnt-- ? 
		(isaac(&m_rc), m_rc.randcnt=(N-1), m_rc.randrsl[m_rc.randcnt]) : 
		m_rc.randrsl[m_rc.randcnt]);
}


template<UINT32 ALPHA>
inline void QTIsaac<ALPHA>::randinit(randctx* ctx, bool bUseSeed)
{
	UINT32 a,b,c,d,e,f,g,h;
	int i;

	a = b = c = d = e = f = g = h = GOLDEN_RATIO;

	UINT32* m = (ctx->randmem);
	UINT32* r = (ctx->randrsl);

	if(!bUseSeed)
	{
		ctx->randa = 0;
		ctx->randb = 0;
		ctx->randc = 0;
	}

	// scramble it
	for(i=0; i < 4; ++i)         
	{
		shuffle(a,b,c,d,e,f,g,h);
	}

	if(bUseSeed) 
	{
		// initialize using the contents of r[] as the seed

		for(i=0; i < N; i+=8)
		{
			a+=r[i  ]; b+=r[i+1]; c+=r[i+2]; d+=r[i+3];
			e+=r[i+4]; f+=r[i+5]; g+=r[i+6]; h+=r[i+7];

			shuffle(a,b,c,d,e,f,g,h);

			m[i  ]=a; m[i+1]=b; m[i+2]=c; m[i+3]=d;
			m[i+4]=e; m[i+5]=f; m[i+6]=g; m[i+7]=h;
		}           

		//do a second pass to make all of the seed affect all of m

		for(i=0; i < N; i += 8)
		{
			a+=m[i  ]; b+=m[i+1]; c+=m[i+2]; d+=m[i+3];
			e+=m[i+4]; f+=m[i+5]; g+=m[i+6]; h+=m[i+7];

			shuffle(a,b,c,d,e,f,g,h);

			m[i  ]=a; m[i+1]=b; m[i+2]=c; m[i+3]=d;
			m[i+4]=e; m[i+5]=f; m[i+6]=g; m[i+7]=h;
		}
	}
	else
	{
		// fill in mm[] with messy stuff

		shuffle(a,b,c,d,e,f,g,h);

		m[i  ]=a; m[i+1]=b; m[i+2]=c; m[i+3]=d;
		m[i+4]=e; m[i+5]=f; m[i+6]=g; m[i+7]=h;

	}

	isaac(ctx);         // fill in the first set of results 
	ctx->randcnt = N;   // prepare to use the first set of results 
}


template<UINT32 ALPHA>
inline UINT32 QTIsaac<ALPHA>::ind(UINT32* mm, UINT32 x)
{
	return (*(UINT32*)((byte*)(mm) + ((x) & ((N-1)<<2))));
}


template<UINT32 ALPHA>
inline void QTIsaac<ALPHA>::rngstep(UINT32 mix, UINT32& a, UINT32& b, UINT32*& mm, UINT32*& m, UINT32*& m2, UINT32*& r, UINT32& x, UINT32& y)
{
	x = *m;  
	a = (a^(mix)) + *(m2++); 
	*(m++) = y = ind(mm,x) + a + b; 
	*(r++) = b = ind(mm,y>>ALPHA) + x; 
}


template<UINT32 ALPHA>
inline void QTIsaac<ALPHA>::shuffle(UINT32& a, UINT32& b, UINT32& c, UINT32& d, UINT32& e, UINT32& f, UINT32& g, UINT32& h)
{ 
	a^=b<<11; d+=a; b+=c; 
	b^=c>>2;  e+=b; c+=d; 
	c^=d<<8;  f+=c; d+=e; 
	d^=e>>16; g+=d; e+=f; 
	e^=f<<10; h+=e; f+=g; 
	f^=g>>4;  a+=f; g+=h; 
	g^=h<<8;  b+=g; h+=a; 
	h^=a>>9;  c+=h; a+=b; 
}


template<UINT32 ALPHA>
inline void QTIsaac<ALPHA>::isaac(randctx* ctx)
{
	UINT32 x,y;

	UINT32* mm = ctx->randmem;
	UINT32* r  = ctx->randrsl;

	UINT32 a = (ctx->randa);
	UINT32 b = (ctx->randb + (++ctx->randc));

	UINT32* m    = mm; 
	UINT32* m2   = (m+(N/2));
	UINT32* mend = m2;

	for(; m<mend; )
	{
		rngstep((a<<13), a, b, mm, m, m2, r, x, y);
		rngstep((a>>6) , a, b, mm, m, m2, r, x, y);
		rngstep((a<<2) , a, b, mm, m, m2, r, x, y);
		rngstep((a>>16), a, b, mm, m, m2, r, x, y);
	}

	m2 = mm;

	for(; m2<mend; )
	{
		rngstep((a<<13), a, b, mm, m, m2, r, x, y);
		rngstep((a>>6) , a, b, mm, m, m2, r, x, y);
		rngstep((a<<2) , a, b, mm, m, m2, r, x, y);
		rngstep((a>>16), a, b, mm, m, m2, r, x, y);
	}

	ctx->randb = b;
	ctx->randa = a;
}


#endif // __ISAAC_HPP

