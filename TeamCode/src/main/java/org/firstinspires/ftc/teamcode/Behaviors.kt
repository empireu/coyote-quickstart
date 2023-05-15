package org.firstinspires.ftc.teamcode

import empireu.coyote.BehaviorCompositeNode
import empireu.coyote.BehaviorContext
import empireu.coyote.BehaviorCreateContext
import empireu.coyote.BehaviorStatus
import kotlin.reflect.KClass

interface ICompositeNode {
    fun <TContainer : Any> load(k: KClass<TContainer>) : TContainer
    fun <TContainer : Any> loadJ(j: Class<TContainer>) : TContainer
}

fun interface ISystemFactory {
    fun create(node: ICompositeNode): ISystem
}

class SystemNode(ctx: BehaviorCreateContext, private val factory: ISystemFactory) : BehaviorCompositeNode(ctx.name, ctx.runOnce, ctx.savedData), ICompositeNode {
    override fun update(context: BehaviorContext): BehaviorStatus {
        val storage = context.getOrStore(StorageKey(this)) {
            StorageValue(factory.create(this).also { it.initialize() })
        }

        return storage.system.update()
    }

    private data class StorageKey(val node: SystemNode)
    private data class StorageValue(val system: ISystem)

    override fun <TContainer : Any> load(k: KClass<TContainer>): TContainer {
        return super.loadStorage(k)
    }

    override fun <TContainer : Any> loadJ(j: Class<TContainer>): TContainer {
        return super.loadStorageJava(j)
    }
}

interface ISystem {
    fun initialize(){}
    fun update(): BehaviorStatus
}