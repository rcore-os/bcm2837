TARGET := aarch64-unknown-none

default:
	cargo build --target $(TARGET)

clippy:
	cargo clippy --target $(TARGET)

clean:
	cargo clean

fmt:
	cargo fmt
