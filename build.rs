use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let target = env::var("TARGET").unwrap_or_default();
    let out = PathBuf::from(env::var_os("OUT_DIR").unwrap());

    // Reserve the last 16 KiB (4 × 4 KiB SPI flash sectors) for keyer
    // settings persistence via sequential-storage.  The firmware FLASH
    // region is shrunk to match so the linker can't push code into the
    // storage range; the `src/storage.rs` constants must agree.
    let memory_x: Option<&[u8]> = match target.as_str() {
        "thumbv6m-none-eabi" => Some(
            b"
MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100 - 16K
    RAM   : ORIGIN = 0x20000000, LENGTH = 264K
}
        ",
        ),
        // RP2350 layout — mirrors embassy-rs/embassy examples/rp235x/memory.x
        // and ../../cw-adapter/rp/build.rs. See that file for the rationale
        // behind the `.start_block` / `.end_block` SECTIONS directives.
        "thumbv8m.main-none-eabihf" => Some(
            br#"
MEMORY {
    FLASH : ORIGIN = 0x10000000, LENGTH = 2048K - 16K
    RAM   : ORIGIN = 0x20000000, LENGTH = 512K
}

SECTIONS {
    .start_block : ALIGN(4)
    {
        __start_block_addr = .;
        KEEP(*(.start_block));
        KEEP(*(.boot_info));
    } > FLASH
} INSERT AFTER .vector_table;

_stext = ADDR(.start_block) + SIZEOF(.start_block);

SECTIONS {
    .end_block : ALIGN(4)
    {
        __end_block_addr = .;
        KEEP(*(.end_block));
    } > FLASH
} INSERT AFTER .uninit;

PROVIDE(start_to_end = __end_block_addr - __start_block_addr);
PROVIDE(end_to_start = __start_block_addr - __end_block_addr);
"#,
        ),
        _ => None,
    };

    if let Some(content) = memory_x {
        let mut file = File::create(out.join("memory.x")).unwrap();
        file.write_all(content).unwrap();
        println!("cargo:rustc-link-search={}", out.display());
    }
}
