
build:
	cargo -v build --features="stm32f303"

doc:
	cargo -v doc --features="stm32f303,rt" --open

.PHONY: build
