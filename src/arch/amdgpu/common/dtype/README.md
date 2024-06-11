# Microscaling Formats

This directory defines [microscaling formats](https://www.opencompute.org/documents/ocp-microscaling-formats-mx-v1-0-spec-final-pdf) which are reduced precision floating point formats.
The class makes some assumptions to simplify things and is not completely generic.
For example:
- Types must be smaller than 32-bits.
- Type conversions currently assume that either:
    - The destination format exponent and mantissa bits are both greater or equal to the source format.
    - OR the destination format exponent and mantissa are both less than or equal to the source format.
    - In other words, one type cannot have larger exponent and smaller mantissa and visa versa.
- Basic MX operations are implementation defined, meaning MX types can be converted to FP32 for arithmetic
    - This means that arithmetic operators need not be defined for MX types.
- Exponent and mantissa of zero is zero. There is no special case for the sign (i.e, -0 is not special).
- The spec does not differentiate between signaling and quiet NaN, therefore quiet NaN is used.
- New types must template specialize the following standard library methods:
    - isinf(T)
    - isnan(T)
    - isnormal(T)
- New types must template specialize the following std::numeric_limits<T> members / methods:
    - has_infinity / infinity()
    - has_quiet_NaN / quiet_NaN()
