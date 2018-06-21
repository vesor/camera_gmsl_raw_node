
typedef void (*IMAGE_CALLBACK)(unsigned long long timestampUs, const unsigned char* image, unsigned int w, unsigned int h);


int run_main(IMAGE_CALLBACK imgCb);
