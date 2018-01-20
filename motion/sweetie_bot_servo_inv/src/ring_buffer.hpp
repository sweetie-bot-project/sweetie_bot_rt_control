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
		void reset(unsigned int delay);

		//next two methods return references to elements in ring buffer

		//returns reference to the delaied element
		const T &pos_to_get();
		//add new elemet and return reference to it, this element is undefined until you assign some value to it
		T &pos_to_put();

		//return the buffer-average value
		T average_elem();

		//initialize buffer using last written value
		void init();
};

template<class T> ring_buffer<T>::ring_buffer(unsigned int delay): _size(delay + 1), curr_pos(0) {
	buf = std::vector<T>(_size);
}


template<class T> unsigned int ring_buffer<T>::delay_value() {

	return _size-1;
}

template<class T> void ring_buffer<T>::reset(unsigned int delay) {

	_size = delay + 1;
	buf.assign(_size, T());
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

template<class T> T ring_buffer<T>::average_elem() {
	T sum;
	unsigned int i;

	i = curr_pos;
	do {

		sum = sum + buf[i];

		i = (i + 1) % _size;
	} while (i != curr_pos);

	return sum / _size;
}

//specific realisation for sweetie_bot_kinematics_msgs::JointStateAccel
template<> sweetie_bot_kinematics_msgs::JointStateAccel ring_buffer<sweetie_bot_kinematics_msgs::JointStateAccel>::average_elem() {
	sweetie_bot_kinematics_msgs::JointStateAccel sum;
	unsigned int i;
	unsigned int j;
	unsigned int n;

	sum.name = buf[curr_pos].name;
	n = sum.name.size();
	sum.position.assign(n, 0);
	sum.velocity.assign(n, 0);
	sum.acceleration.assign(n, 0);
	sum.effort.assign(n, 0);

	i = curr_pos;
	do {

		//skip incorrect joints
		if (buf[i].name.size() == n && buf[i].position.size() == n
				&& buf[i].velocity.size() == n && buf[i].acceleration.size() == n
								&& buf[i].effort.size() == n) {

			for (j = 0; j < n; j++) {

				sum.position[j] = sum.position[j] + buf[i].position[j];
				sum.velocity[j] = sum.velocity[j] + buf[i].velocity[j];
				sum.acceleration[j] = sum.acceleration[j] + buf[i].acceleration[j];
				sum.effort[j] = sum.effort[j] + buf[i].effort[j];
			}
		}

		i = (i + 1) % _size;
	} while (i != curr_pos);

	for (j = 0; j < n; j++) {
		sum.position[j] = sum.position[j] / _size;
		sum.velocity[j] = sum.velocity[j] / _size;
		sum.acceleration[j] = sum.acceleration[j] / _size;
		sum.effort[j] = sum.effort[j] / _size;
	}

	return sum;
}

template<class T> void ring_buffer<T>::init() {
	unsigned int i, j;

	if (curr_pos == 0)
		j = _size;
	else
		j = curr_pos;

	i = curr_pos;
	do {

		buf[i] = buf[j - 1];

		i = (i + 1) % _size;
	} while (i != curr_pos);

	return;
}
#endif
