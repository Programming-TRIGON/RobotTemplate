package frc.trigon.robot.commands.commandclasses.rumblecommands.rumbleconfigurations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RumbleChain implements RumbleConfiguration {
    private SingleRumble singleRumble;
    private double delaySeconds;
    private RumbleChain nextRumbleLink;

    private RumbleChain(SingleRumble singleRumble, double delaySeconds, RumbleChain nextRumbleLink) {
        this.singleRumble = singleRumble;
        this.delaySeconds = delaySeconds;
        this.nextRumbleLink = nextRumbleLink;
    }

    public Command getRumbleCommand() {
        return nextRumbleLink == null ?
                getLinkRumbleCommand() :
                new SequentialCommandGroup(
                        getLinkRumbleCommand(),
                        new WaitCommand(singleRumble.getDurationSeconds() + delaySeconds),
                        nextRumbleLink.getRumbleCommand()
                );
    }

    public Command getLinkRumbleCommand() {
        return singleRumble.getRumbleCommand();
    }

    public SingleRumble getSingleRumble() {
        return singleRumble;
    }

    public double getDelaySeconds() {
        return delaySeconds;
    }

    public RumbleChain getNextRumbleLink() {
        return nextRumbleLink;
    }

    public void setSingleRumble(SingleRumble singleRumble) {
        this.singleRumble = singleRumble;
    }

    public void setDelaySeconds(double delaySeconds) {
        this.delaySeconds = delaySeconds;
    }

    public void setNextRumbleLink(RumbleChain nextRumbleLink) {
        this.nextRumbleLink = nextRumbleLink;
    }

    public static RumbleChain constructRumbleChainFromLinks(RumbleChainLink... rumbleLinks) {
        final RumbleChain rumbleChainHead = new RumbleChain(
                rumbleLinks[0].singleRumble(),
                rumbleLinks[0].delaySeconds(),
                null
        );
        RumbleChain previousNode = rumbleChainHead;

        for (int i = 1; i < rumbleLinks.length; i++) {
            final RumbleChain currentNode = new RumbleChain(
                    rumbleLinks[i].singleRumble(),
                    rumbleLinks[i].delaySeconds(),
                    null
            );

            previousNode.setNextRumbleLink(currentNode);
            previousNode = currentNode;
        }

        return rumbleChainHead;
    }

    public record RumbleChainLink(SingleRumble singleRumble, double delaySeconds) {
    }
}
