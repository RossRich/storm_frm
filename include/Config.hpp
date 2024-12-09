#define VCC_REF 4.98f //< Напряжение питание платы
#define ADC_MAX 1024u //< Максимальное значение АЦП
#define ADC_CONVERT_FACTOR (VCC_REF / static_cast<float>(ADC_MAX)) //< Значение для преобразования измерения АЦП