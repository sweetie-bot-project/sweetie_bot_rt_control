#ifndef __RING_BUFFER_FOR_HERKULEX_CONTROL_HPP
#define __RING_BUFFER_FOR_HERKULEX_CONTROL_HPP

//T must have operator+() and operator/() or you must specify average_elem() method for your type T
template<class T> class ring_buffer {
	std::vector<T> buf;
	unsigned int _size;
	unsigned int curr_pos;

	public:
		ring_buffer(unsigned int delay = 0);
		unsigned int delay_value() const;
		void reset(unsigned int delay, const T &samp = T());

		//next two methods return references to elements in ring buffer

		//returns reference to the delaied element
		const T &pos_to_get() const;
		//add new elemet and return reference to it, this element is undefined until you assign some value to it
		T &pos_to_put();

		//return reference to last put element
		T &pos_to_reput();

		//return ref to i-th elemnt of buffer started from curr_pos (if i > _size then return (i+curr_pos) % _size)
		const T& operator[] (unsigned int i) const;
};

template<class T> ring_buffer<T>::ring_buffer(unsigned int delay): _size(delay + 1), curr_pos(0) {
	buf = std::vector<T>(_size);
}


template<class T> unsigned int ring_buffer<T>::delay_value() const {

	return _size-1;
}

template<class T> void ring_buffer<T>::reset(unsigned int delay, const T &samp) {

	_size = delay + 1;
	buf.assign(_size, samp);
	curr_pos = 0;
}

template<class T> const T &ring_buffer<T>::pos_to_get() const {

	return buf[curr_pos];
}

template<class T> T &ring_buffer<T>::pos_to_put() {
	T &out = buf[curr_pos];

	curr_pos = (curr_pos + 1) % _size;
	return out;
}

template<class T> T &ring_buffer<T>::pos_to_reput() {

	return buf[(curr_pos + (_size - 1)) % _size];
}

template<class T> const T& ring_buffer<T>::operator[](unsigned int i) const{

	return buf[(curr_pos + (_size - 1 - (i % _size))) % _size];
}
#endif
