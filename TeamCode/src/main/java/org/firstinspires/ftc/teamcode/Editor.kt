package org.firstinspires.ftc.teamcode

import android.os.Environment
import com.google.gson.Gson
import empireu.coyote.JsonProject
import org.firstinspires.ftc.robotcore.internal.android.dex.util.FileUtils

fun getRobotPath(file: String): String {
    val dir = Environment.getExternalStorageDirectory()
    val storagePath: String = dir.absolutePath
    return "$storagePath/$file"
}

val robotCoyotePath get() = getRobotPath("coyote.awoo")

fun loadRobotCoyoteProject(): JsonProject =
    Gson().fromJson(
        FileUtils.readFile(robotCoyotePath).decodeToString(),
        JsonProject::class.java
    )