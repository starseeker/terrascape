#ifndef ARRAY_INCLUDED
#define ARRAY_INCLUDED

#include <cstdlib>

template<class T>
class array {
protected:
    T *data;
    int len;

public:
    array() { data = nullptr; len = 0; }
    
    array(int size) {
        len = size;
        data = new T[size];
        for (int i = 0; i < size; i++) {
            data[i] = T();
        }
    }
    
    virtual ~array() {
        delete[] data;
    }
    
    array(const array& a) {
        len = a.len;
        data = new T[len];
        for (int i = 0; i < len; i++) {
            data[i] = a.data[i];
        }
    }
    
    array& operator=(const array& a) {
        if (this != &a) {
            delete[] data;
            len = a.len;
            data = new T[len];
            for (int i = 0; i < len; i++) {
                data[i] = a.data[i];
            }
        }
        return *this;
    }
    
    T& ref(int i) { return data[i]; }
    const T& ref(int i) const { return data[i]; }
    
    T& operator[](int i) { return data[i]; }
    const T& operator[](int i) const { return data[i]; }
    
    int length() const { return len; }
    int maxLength() const { return len; }
    
    void resize(int new_size) {
        if (new_size <= len) return;
        
        T* new_data = new T[new_size];
        for (int i = 0; i < len; i++) {
            new_data[i] = data[i];
        }
        for (int i = len; i < new_size; i++) {
            new_data[i] = T();
        }
        
        delete[] data;
        data = new_data;
        len = new_size;
    }
};

template<class T>
class array2 {
private:
    T *data;
    int width, height;

public:
    array2() { data = nullptr; width = height = 0; }
    
    array2(int w, int h) {
        width = w;
        height = h;
        data = new T[w * h];
        for (int i = 0; i < w * h; i++) {
            data[i] = T();
        }
    }
    
    virtual ~array2() {
        delete[] data;
    }
    
    array2(const array2& a) {
        width = a.width;
        height = a.height;
        int size = width * height;
        data = new T[size];
        for (int i = 0; i < size; i++) {
            data[i] = a.data[i];
        }
    }
    
    array2& operator=(const array2& a) {
        if (this != &a) {
            delete[] data;
            width = a.width;
            height = a.height;
            int size = width * height;
            data = new T[size];
            for (int i = 0; i < size; i++) {
                data[i] = a.data[i];
            }
        }
        return *this;
    }
    
    void init(int w, int h) {
        delete[] data;
        width = w;
        height = h;
        data = new T[w * h];
        for (int i = 0; i < w * h; i++) {
            data[i] = T();
        }
    }
    
    T& operator()(int x, int y) { 
        return data[y * width + x]; 
    }
    
    const T& operator()(int x, int y) const { 
        return data[y * width + x]; 
    }
    
    int getWidth() const { return width; }
    int getHeight() const { return height; }
};

#endif