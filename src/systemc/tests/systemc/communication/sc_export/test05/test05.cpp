#include "systemc.h"

typedef sc_biguint<121> atom;  // Value to be pipe delayed.

//==============================================================================
// esc_dpipe<T,N> - DELAY PIPELINE FOR AN ARBITRARY CLASS:
//==============================================================================
template<class T, int N>
SC_MODULE(esc_dpipe) {
  public:
    typedef sc_export<sc_signal_inout_if<T> > in;   // To pipe port type.
    typedef sc_export<sc_signal_in_if<T> >    out;  // From pipe port type.

  public:
    SC_CTOR(esc_dpipe)
    {
        m_in(m_pipe[0]);
        m_out(m_pipe[N-1]);
        SC_METHOD(rachet);
        sensitive << m_clk.pos();
    }

  protected:
    void rachet()
    {
        for ( int i = N-1; i > 0; i-- )
        {
            m_pipe[i].write(m_pipe[i-1].read());
        }
    }

  public:
    sc_in_clk m_clk;  // Pipe synchronization.
    in        m_in;   // Input to delay pipe.
    out       m_out;  // Output from delay pipe.

  protected:
    sc_signal<T> m_pipe[N]; // Pipeline stages.
};


// Testbench reader of values from the pipe:

SC_MODULE(Reader)
{
    SC_CTOR(Reader)
    {
        SC_METHOD(extract)
        sensitive << m_clk.pos();
        dont_initialize();
    }

    void extract()
    {
        cout << sc_time_stamp() << ": " << m_from_pipe.read() << endl;
    }

    sc_in_clk    m_clk;         // Module synchronization.
    sc_in<atom > m_from_pipe;   // Output from delay pipe.
};

            

// Testbench writer of values to the pipe:

SC_MODULE(Writer)
{
    SC_CTOR(Writer)
    {
        SC_METHOD(insert)
        sensitive << m_clk.pos();
        m_counter = 0;
    }

    void insert()
    {
        m_to_pipe.write(m_counter);
        m_counter++;
    }

    sc_in_clk       m_clk;       // Module synchronization.
    atom            m_counter;   // Write value.
    sc_inout<atom > m_to_pipe;   // Input for delay pipe.
};

// Main program

int sc_main(int argc, char* argv[])
{
    sc_clock          clock;
    esc_dpipe<atom,4> delay("pipe");
    Reader            reader("reader");
    Writer            writer("writer");

    delay.m_clk(clock);

    reader.m_clk(clock);
    reader.m_from_pipe(delay.m_out);

    writer.m_clk(clock);
    writer.m_to_pipe(delay.m_in);

    sc_start(50, SC_NS);

    return 0;
}
