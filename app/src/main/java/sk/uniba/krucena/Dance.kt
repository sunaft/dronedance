package sk.uniba.krucena

import android.content.Context
import android.util.Log
import java.io.File

/** holds data for one performance
 *  format of the dance config file (each performance has separate object):
 *
 *  number_of_performances=<number>
 *  <time_ms> <cmd> <cmd_args>
 *  # full line comments
 *
 *  where <cmd> is one of with the corresponding arguments:
 *
 *  takeoff
 *  land
 *  fly <fly_arguments>
 *  pos <expected_x> <expected_y> <expected_z> <expected_yaw>
 *  leds on|off
 *  run <name_of_procedure>
 *  refpoint <x> <y> <z> <yaw>
 *  relative <x> <y> <z> <yaw>
 *  absolute
 *  hspeed <hspeedlimit>
 *  vspeed <vspeedlimit>
 *  end
 *
 *  procedure <name_of_procedure> [<options>]
 *      ...commands...
 *  endp
 *
 *  Note: times in procedure body are relative to when it was called
 *        recursion is currently not supported
 *
 * where
 *
 * <fly_arguments> ::= <pitch> <roll> <yaw> <altitude> <altitude_mode> <rollpitch_mode> <yaw_mode> <rollpitch_system>
 *
 * where
 *
 * <altitude_mode> ::= ALT_m | ALT_velo              # either height in m or velocity
 * <rollpitch_mode> ::= RP_deg | RP_velo | RP_pos    # either angle or velocity mode
 * <yaw_mode> ::= Y_deg | Y_velo                     # either angle or velocity mode
 * <rollpitch_system> ::= Ground | Body              # coordinate system for roll and pitch values
 *
 * options working on Mini 3:
 *    ALT_m - use <altitude> as approximate distance from ground
 *    AlT_velo - use <altitude> as speed (probably in m/s) => danger
 *    Y_deg - set yaw based on compass (a bit unreliable in metal building)
 *    Y_velo - rotates in yaw axis in given speed angle/s
 *    RP_velo - moves in given speed m/s in roll/pitch directions
 *    RP_deg - tilts the aircraft in roll/pitch direction by specified degrees => danger (2 deg is already steady movement)
 *    RP_pos - not sure if this is supported or if it does anything
 *    Body - roll and pitch movement are relative to world coordinates (determined by compass)
 *    Ground - roll and pitch movement are relative to plane (pitch = forward, roll = sideways)
 *
 *    refpoint sets the reference point for all the forthcoming POS commands, if relative mode is on
 *             in relative mode, the POS coordinates are calculated relatively to current REFPOINT
 *             and then added to current relative point set with RELATIVE command
 *
 *    relative turns on relative mode, and sets up the current relative point
 *    absolute turns off relative mode, reference point will be ignored, and POS coordinates used as they are
 *
 *    hspeedlimit and vspeedlimit are in m/s, default 0.3 and 1.0 resp.
 */

enum class DanceInstructionKind {
    FLY, POS, LEDS, END, TAKEOFF, LAND, RUN, NONE, REFPOINT, RELATIVE, ABSOLUTE, HSPEED, VSPEED
}

enum class InstructionPosValue {
    POS, VELOCITY
}
enum class InstructionAngularValue {
    DEG, VELOCITY, POS
}
enum class InstructionCoordSystem {
    GROUND, BODY
}
open class DanceInstructionArguments() {
}

class FlyArguments(val pitch : Float, val roll : Float, val yaw : Float, val altitude : Float,
                   val altMode : InstructionPosValue, val RPMode : InstructionAngularValue,
                   val yawMode : InstructionAngularValue, val coordSystem : InstructionCoordSystem)
          : DanceInstructionArguments() {
}

data class PosArguments(var x: Float, var y: Float, var z: Float, var yaw: Float)
          : DanceInstructionArguments() {
}

class LedsArguments(val show: Boolean)
          : DanceInstructionArguments() {
}

class RunArguments(val name: String)
          : DanceInstructionArguments() {
}

class SpeedArguments(val limit: Double) : DanceInstructionArguments() {
}

class DanceCommand(val time : Int,
                   val kind : DanceInstructionKind = DanceInstructionKind.NONE,
                   val arguments : DanceInstructionArguments? = null) {
}

class Dance
{
    val instructions : MutableList<DanceCommand> = ArrayList()

    val procedures : MutableMap<String, Dance> = HashMap()

    var landingTime : Int = 0

    fun append(instruction: DanceCommand) {
        instructions.add(instruction)
    }

    fun get(index : Int) : DanceCommand {
        return instructions[index];
    }

    companion object {
        val practiceRun : Boolean = false     // set to true to always run dance_0 instead of dance_id
        var nameOfProcedure : String? = null    // if set, we are parsing middle of this procedure
        var constructedProcedure : Dance? = null    // here we collect the commands of the procedure

        private fun loadDance(lines : List<String>, lnCounter : Int) : Pair<Dance?, Int>
        {
            val d = Dance()
            // each dance starts with None command so that the processCommand has something to do
            d.append(DanceCommand(0, DanceInstructionKind.NONE, DanceInstructionArguments()))

            var lastTime = 0
            var saveLastTime = 0
            var lnInd = lnCounter

            var currentDance : Dance? = d

            while(true) {
                if (lines[lnInd].length == 0) // empty lines, skip
                {
                    lnInd++
                    continue
                }
                if (lines[lnInd][0] == '#') // comments, skip
                {
                    lnInd++
                    continue
                }

                // treat multiple spaces and other white space properly
                val ln = lines[lnInd].trim().split(Regex("\\s+")).map { it.uppercase() }
                lnInd++

                if (ln.size == 0) continue

                val stepTime = ln[0].toIntOrNull() ?: -1
                if ((stepTime < lastTime) && (stepTime >= 0))
                    Log.e("Config", "time is decreasing at line ${lnInd}")
                lastTime = stepTime

                if (ln[0] == "PROCEDURE")
                {
                    nameOfProcedure = ln[1];
                    constructedProcedure = Dance()
                    currentDance = constructedProcedure
                    saveLastTime = lastTime
                    lastTime = 0
                    currentDance?.append(DanceCommand(0, DanceInstructionKind.NONE, DanceInstructionArguments()))
                }
                else if (ln[0] == "ENDP")
                {
                    nameOfProcedure?.let { name ->
                        constructedProcedure?.let { proc ->
                            d.procedures.put(name, proc)
                        }
                    }
                    nameOfProcedure = null
                    currentDance = d
                    lastTime += saveLastTime
                }
                else if (ln[1] == "LEDS")
                {
                    val showLEDs : Boolean = ln[2] == "ON"
                    currentDance?.append(DanceCommand(stepTime,
                                          DanceInstructionKind.LEDS,
                                          LedsArguments(showLEDs)))
                }
                else if (ln[1] == "POS")
                {
                    val x : Float = ln[2].toFloat()
                    val y : Float = ln[3].toFloat()
                    val z : Float = ln[4].toFloat()
                    val yaw : Float = ln[5].toFloat()
                    currentDance?.append(DanceCommand(stepTime,
                                          DanceInstructionKind.POS,
                                          PosArguments(x, y, z, yaw)))
                }
                else if (ln[1] == "FLY")
                {
                    val pitch : Float = ln[2].toFloat()
                    val roll : Float = ln[3].toFloat()
                    val yaw : Float = ln[4].toFloat()
                    val altitude : Float = ln[5].toFloat()
                    val altMode : InstructionPosValue = if (ln[6] == "ALT_M") InstructionPosValue.POS else InstructionPosValue.VELOCITY
                    val RPMode : InstructionAngularValue = if (ln[7] == "RP_DEG") InstructionAngularValue.DEG
                                                           else if (ln[7] == "RP_VELO") InstructionAngularValue.VELOCITY
                                                           else InstructionAngularValue.POS
                    val yawMode : InstructionAngularValue = if (ln[8] == "Y_DEG") InstructionAngularValue.DEG else InstructionAngularValue.VELOCITY
                    val coordSystem : InstructionCoordSystem = if (ln[9] == "GROUND") InstructionCoordSystem.GROUND else InstructionCoordSystem.BODY
                    currentDance?.append(DanceCommand(stepTime,
                                          DanceInstructionKind.FLY,
                                          FlyArguments(pitch, roll, yaw, altitude,
                                                       altMode, RPMode, yawMode, coordSystem)))
                }
                else if (ln[1] == "TAKEOFF")
                {
                    currentDance?.append(DanceCommand(stepTime, DanceInstructionKind.TAKEOFF, DanceInstructionArguments()))
                }
                else if (ln[1] == "LAND")
                {
                    currentDance?.append(DanceCommand(stepTime, DanceInstructionKind.LAND, DanceInstructionArguments()))
                    currentDance?.landingTime = stepTime
                }
                else if (ln[1] == "RUN")
                {
                    currentDance?.append(DanceCommand(stepTime, DanceInstructionKind.RUN, RunArguments(ln[2])))
                }
                else if (ln[1] == "REFPOINT")
                {
                    val x : Float = ln[2].toFloat()
                    val y : Float = ln[3].toFloat()
                    val z : Float = ln[4].toFloat()
                    val yaw : Float = ln[5].toFloat()
                    currentDance?.append(DanceCommand(stepTime,
                        DanceInstructionKind.REFPOINT,
                        PosArguments(x, y, z, yaw)))
                }
                else if (ln[1] == "RELATIVE")
                {
                    val x : Float = ln[2].toFloat()
                    val y : Float = ln[3].toFloat()
                    val z : Float = ln[4].toFloat()
                    val yaw : Float = ln[5].toFloat()
                    currentDance?.append(DanceCommand(stepTime,
                        DanceInstructionKind.RELATIVE,
                        PosArguments(x, y, z, yaw)))
                }
                else if (ln[1] == "ABSOLUTE")
                {
                    currentDance?.append(DanceCommand(stepTime, DanceInstructionKind.ABSOLUTE))
                }
                else if (ln[1] == "HSPEED")
                {
                    currentDance?.append(DanceCommand(stepTime, DanceInstructionKind.HSPEED, SpeedArguments(ln[2].toDouble())))
                }
                else if (ln[1] == "VSPEED")
                {
                    currentDance?.append(DanceCommand(stepTime, DanceInstructionKind.VSPEED, SpeedArguments(ln[2].toDouble())))
                }
                else if (ln[1] == "END")
                {
                    currentDance?.append(DanceCommand(stepTime, DanceInstructionKind.END))
                    break;
                }
                else Log.e("Config", "ignoring unrecognized command ${ln[1]} at line ${lnInd}")
            }
            return Pair(d, lnInd)
        }

        fun load(context: Context, droneId : Int) : MutableList<Dance>
        {
            val dances : MutableList<Dance> = ArrayList()

            try {
                val danceId = if (practiceRun) 0 else droneId;
                var file = File(context.filesDir, "dance_" + danceId + ".txt")
                if (!file.exists()) file = File(context.filesDir, "dance_0.txt")

                Log.i("Config", "Looking for dance config in: ${context.filesDir.absolutePath}")

                Log.i("Config", "dance config load")
                val lines = file.readLines()
                Log.i("Config", "dance config readLines ok")
                var lnCounter = 0
                var numberOfPerformances = 0
                while (true) {
                    val aLine = lines[lnCounter].trim()

                    if (aLine.isEmpty())
                    {
                        lnCounter++;
                        continue
                    }
                    if (aLine.startsWith('#'))
                    {
                        lnCounter++
                        continue
                    }
                    //Log.i("Config", "before: ${lines[lnCounter]}")
                    val firstLine = aLine.split(Regex("\\s+")).map { it.uppercase() }
                    //Log.i("Config", "after: ${firstLine}")

                    if (firstLine.size == 0) {
                        lnCounter++
                        continue
                    }
                    if (!firstLine[0].equals("NUMBER_OF_PERFORMANCES") ||
                        !firstLine[1].equals("=")) {
                        Log.e("Config", "dance file should start with 'number_of_performances = N', but '${firstLine}' found");
                        return dances;
                    }
                    numberOfPerformances = Integer.parseInt(firstLine[2])
                    Log.i("Config", "number of performances: ${numberOfPerformances}")
                    lnCounter++
                    break
                }

                for (perf in 1..numberOfPerformances) {
                    Log.i("Config", "loading performance #{$perf}")
                    val (dance, newLnCounter) = loadDance(lines, lnCounter)
                    lnCounter = newLnCounter
                    if (dance == null) {
                        Log.e(
                            "Config",
                            "problem loading the performance #{$perf}"
                        );
                        return dances;
                    }
                    dances.add(dance)
                }
            } catch (e: Exception) {
                Log.e("Config", "Failed to read dance config file: ${e.message}")
                return dances
            }
            return dances
        }
    }
}