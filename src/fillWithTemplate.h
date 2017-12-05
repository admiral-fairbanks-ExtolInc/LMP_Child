template <class T> void Fill_With (T *target_array, uint8_t array_size, T value) {
    for(uint8_t l = 0; l < array_size; l++){
        target_array[l] = value;
    }
}
