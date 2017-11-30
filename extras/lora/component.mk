# Component makefile for extras/lora

# expected anyone using bmp driver includes it as 'lora/lora.h'
INC_DIRS += $(lora_ROOT)..

# args for passing into compile rule generation
lora_SRC_DIR =  $(lora_ROOT)

$(eval $(call component_compile_rules,lora))
