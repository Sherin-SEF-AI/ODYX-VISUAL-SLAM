# Keep JNI entry points (names must match the C++ symbol names).
-keepclasseswithmembernames class * {
    native <methods>;
}
-keep class ai.deepmost.odyx.jni.** { *; }
