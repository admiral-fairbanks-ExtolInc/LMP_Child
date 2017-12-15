template <class T> void fillWith (T *targetArray, uint8_t arraySize, T value) {
    for(uint8_t l = 0; l < arraySize; l++){
        targetArray[l] = value;
    }
}
