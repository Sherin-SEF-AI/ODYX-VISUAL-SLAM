import java.util.Properties

plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.kotlin.android)
    alias(libs.plugins.kotlin.compose)
    alias(libs.plugins.ksp)
}

// Resolve the OpenCV Android SDK location (official prebuilt, Apache-2.0).
// Order: -PopencvSdk=... > gradle.properties odyx.opencv.sdk > ODYX_OPENCV_SDK env.
val opencvSdkDir: String = run {
    (project.findProperty("opencvSdk") as String?)
        ?: (project.findProperty("odyx.opencv.sdk") as String?)?.takeIf { it.isNotBlank() }
        ?: System.getenv("ODYX_OPENCV_SDK")
        ?: ""
}
val nativeJobs: String =
    (project.findProperty("odyx.native.jobs") as String?)?.takeIf { it.isNotBlank() } ?: "4"

android {
    namespace = "ai.deepmost.odyx"
    compileSdk = 35
    ndkVersion = "26.3.11579264"   // r26d (installed via sdkmanager)

    defaultConfig {
        applicationId = "ai.deepmost.odyx"
        minSdk = 26
        targetSdk = 35
        versionCode = 1
        versionName = "0.1.0"

        ndk {
            // Prototype targets arm64-v8a. armeabi-v7a can be added but GTSAM
            // cross-compile is only validated for arm64.
            abiFilters += listOf("arm64-v8a")
        }

        externalNativeBuild {
            cmake {
                arguments += listOf(
                    "-DANDROID_STL=c++_shared",
                    "-DODYX_OPENCV_SDK=$opencvSdkDir",
                    "-DODYX_NATIVE_JOBS=$nativeJobs",
                    // pass the resolved NDK to the in-tree ExternalProject builds
                    "-DODYX_ABI=arm64-v8a",
                    "-DODYX_ANDROID_PLATFORM=android-26"
                )
                cppFlags += "-std=c++17"
            }
        }
    }

    externalNativeBuild {
        cmake {
            path = file("src/main/cpp/CMakeLists.txt")
            version = "3.22.1"
        }
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(getDefaultProguardFile("proguard-android-optimize.txt"), "proguard-rules.pro")
        }
        debug {
            isJniDebuggable = true
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    kotlinOptions {
        jvmTarget = "17"
    }

    buildFeatures {
        compose = true
        buildConfig = true
    }

    androidResources {
        // The 49 MB ORB vocabulary must be stored UNCOMPRESSED, otherwise
        // AssetManager.open() fails on large compressed assets (FileNotFoundException).
        noCompress += "dbow3"
    }

    // Kotlin sources live under src/main/kotlin (in addition to default java dir).
    sourceSets["main"].kotlin.srcDir("src/main/kotlin")

    packaging {
        // OpenCV ships libopencv_java4.so; we link the native static/shared world
        // ourselves. Keep one copy of c++_shared.
        jniLibs {
            pickFirsts += listOf("**/libc++_shared.so")
        }
        resources {
            excludes += "/META-INF/{AL2.0,LGPL2.1}"
        }
    }
}

dependencies {
    implementation(libs.androidx.core.ktx)
    implementation(libs.androidx.lifecycle.runtime.ktx)
    implementation(libs.androidx.lifecycle.runtime.compose)
    implementation(libs.androidx.lifecycle.service)
    implementation(libs.androidx.activity.compose)

    implementation(platform(libs.androidx.compose.bom))
    implementation(libs.androidx.ui)
    implementation(libs.androidx.ui.graphics)
    implementation(libs.androidx.ui.tooling.preview)
    implementation(libs.androidx.material3)
    implementation(libs.androidx.material.icons.extended)
    debugImplementation(libs.androidx.ui.tooling)

    implementation(libs.androidx.camera.core)
    implementation(libs.androidx.datastore.preferences)

    implementation(libs.androidx.room.runtime)
    implementation(libs.androidx.room.ktx)
    ksp(libs.androidx.room.compiler)

    implementation(libs.play.services.location)
    implementation(libs.kotlinx.coroutines.android)
    implementation(libs.timber)
    implementation(libs.accompanist.permissions)
}

// Fail fast with a clear message if OpenCV SDK isn't configured.
tasks.matching { it.name.contains("externalNativeBuild", ignoreCase = true) }.configureEach {
    doFirst {
        if (opencvSdkDir.isBlank()) {
            throw GradleException(
                "OpenCV Android SDK not configured. Run scripts/fetch_deps.sh --opencv " +
                "then set odyx.opencv.sdk=<sdk>/sdk/native/jni in gradle.properties " +
                "(or pass -PopencvSdk=...)."
            )
        }
    }
}
