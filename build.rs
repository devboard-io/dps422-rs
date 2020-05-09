use std::env;
use std::fs;
use std::path::PathBuf;
fn main() {

    let memory_layout = "stm32g070.x";

    if memory_layout.len() > 0 {
        let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
        println!("cargo:rustc-link-search={}", out.display());
        println!("selected file: {}", memory_layout);
        fs::copy(memory_layout, out.join("memory.x")).unwrap();
        println!("cargo:rerun-if-changed={}", memory_layout);
    }
    println!("cargo:rerun-if-changed=build.rs");
}
