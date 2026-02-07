package dev.nextftc.core.commands.conditionals

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.NullCommand

/**
 * This command behaves as an `if` statement, and schedules commands based on the result of the if
 * statement. It is blocking, meaning `isDone` will not return `true` until the scheduled command
 * has completed running.
 * @param condition the condition to reference
 * @param trueCommand the command to schedule if the reference is true
 * @param falseCommand the command to schedule if the reference is false
 */
class WGIfElseCommand @JvmOverloads constructor(
    private val condition: () -> Boolean,
    private val trueCommand: Command,
    private val falseCommand: Command = NullCommand()
) : Command() {

    private lateinit var selectedCommand: Command

    override val isDone: Boolean get() = selectedCommand.isDone

    override fun start() {
        selectedCommand = if (condition()) trueCommand else falseCommand
        selectedCommand.start()
    }

    override fun update() = selectedCommand.update()

    override fun stop(interrupted: Boolean) = selectedCommand.stop(interrupted)
}