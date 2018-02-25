#ifndef __RING_BUFFER_FOR_HERKULEX_CONTROL_HPP
#define __RING_BUFFER_FOR_HERKULEX_CONTROL_HPP

//T must have operator+() and operator/() or you must specify average_elem() method for your type T
template<class T> class ring_buffer {
	std::vector<T> buf;
	unsigned int _size;
	unsigned int curr_pos;

	public:
		ring_buffer(unsigned int delay = 0);
		unsigned int delay_value();
		void reset(unsigned int delay, T samp = T());

		//next two methods return references to elements in ring buffer

		//returns reference to the delaied element
		const T &pos_to_get();
		//add new elemet and return reference to it, this element is undefined until you assign some value to it
		T &pos_to_put();

		//return ref to i-th elemnt of buffer started from curr_pos (if i > _size then return (i+curr_pos) % _size) 
		T& operator[] (unsigned int i);
};

template<class T> ring_buffer<T>::ring_buffer(unsigned int delay): _size(delay + 1), curr_pos(0) {
	buf = std::vector<T>(_size);
}


template<class T> unsigned int ring_buffer<T>::delay_value() {

	return _size-1;
}

template<class T> void ring_buffer<T>::reset(unsigned int delay, T samp) {

	_size = delay + 1;
	buf.assign(_size, samp);
	curr_pos = 0;
}

template<class T> const T &ring_buffer<T>::pos_to_get() {

	return buf[curr_pos];
}

template<class T> T &ring_buffer<T>::pos_to_put() {
	T &out = buf[curr_pos];

	curr_pos = (curr_pos + 1) % _size;
	return out;
}

template<class T> T& ring_buffer<T>::operator[](unsigned int i) {

	return buf[(curr_pos + i) % _size];
}
#endif
