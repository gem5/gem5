/**
 * @file
 * Dummy definitions of STL classes to pick up relationships in doxygen.
 */

namespace std {

/** STL vector class*/
template <class T> class vector {
  public:
    /** Dummy Item */
    T item;
};

/** STL deque class */
template <class T> class deque {
  public:
    /** Dummy Item */
    T item;
};

/** STL list class */
template <class T> class list {
  public:
    /** Dummy Item */
    T item;
};

/** STL pair class */
template <class X, class Y> class pair {
  public:
    /** Dummy Item */
    X item1;
    /** Dummy Item */
    Y item2;
};

}
