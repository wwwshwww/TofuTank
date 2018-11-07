package final17d;

import java.awt.Color;
import java.awt.geom.Point2D;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.Set;
import robocode.*;
import static robocode.util.Utils.*;

/**
 *
 * @author C0116129
 */
public class C0116129 extends AdvancedRobot {

    Map<String, EnemyRobot> enemies;
    Set<String> Ekeys;
    EnemyRobot tag;
    Random rnd;
    double centerX, centerY;
    double avgX, avgY;
    double avgDis;
    double firePower;
    int hitCount;
    int moveDir;
    int scanDir;
    long changeTime;
    final int FORWARD = 1;
    final int BACKWARD = -1;
    final int HIT_COUNT_MAX = 12;
    final int CHANGE_RATE = 15;
    final double FIREPOWER_MAX = 3.0;
    final double FIREPOWER_MIN = 0.1;
    final double GUESS_FULL = 1;
    final double GUESS_DECREASE_AMOUNT = 0.5;
    final double SENTRY_TOLERANCE = 54.0;

    @Override
    public void run() {
        setColors(Color.white, Color.lightGray, Color.lightGray, Color.black, Color.white);
        setAdjustRadarForGunTurn(true);
        setAdjustGunForRobotTurn(true);
        setAdjustRadarForRobotTurn(true);
        enemies = new HashMap<>();
        tag = new EnemyRobot();
        rnd = new Random();
        centerX = getBattleFieldWidth() / 2;
        centerY = getBattleFieldHeight() / 2;
        hitCount = HIT_COUNT_MAX;
        moveDir = FORWARD;
        scanDir = FORWARD;
        turnRadarRight(360);
        while (true) {
            setTag();
            if (getOthers() == 1) {
                radarOnOne();
            } else {
                setTurnRadarRight(360);
            }
            checkTagFired();
            decideFireP(tag.distance);
            guessGun();
            if (checkSentry()) {
                moveOnSentry();
            } else {
                moveTo(90 + (15 * moveDir));
            }
            execute();
        }
    }

    @Override
    public void onScannedRobot(ScannedRobotEvent e) {
        EnemyRobot ene;
        double changePerTime;
        double changeSum;
        double absBear;
        double absBearRad;
        if (enemies.containsKey(e.getName())) {
            ene = enemies.get(e.getName());
        } else {
            ene = new EnemyRobot();
            enemies.put(e.getName(), ene);
            ene.guessAccuracy = GUESS_FULL;
            ene.hitCount = HIT_COUNT_MAX;
            ene.heading = e.getHeadingRadians();
            ene.energy = e.getEnergy();
            ene.changehead = 0.0;
        }
        ene.preChangehead = ene.changehead;
        ene.preEnergy = ene.energy;

        changeSum = normalRelativeAngle(e.getHeadingRadians() - ene.heading);
        changePerTime = changeSum / (getTime() - ene.time);
        ene.changehead = changePerTime;

        absBear = normalAbsoluteAngleDegrees(getHeading() + e.getBearing());
        absBearRad = Math.toRadians(absBear);
        ene.X = getX() + e.getDistance() * Math.sin(absBearRad);
        ene.Y = getY() + e.getDistance() * Math.cos(absBearRad);

        ene.name = e.getName();
        ene.distance = e.getDistance();
        ene.bearing = e.getBearing();
        ene.heading = e.getHeadingRadians();
        ene.energy = e.getEnergy();
        ene.velocity = e.getVelocity();
        ene.time = getTime();
        ene.fired = ene.preEnergy - ene.energy >= FIREPOWER_MIN
                && ene.preEnergy - ene.energy <= FIREPOWER_MAX;
    }

    public void setTag() {
        EnemyRobot ene;
        String nearTank = "";
        double nearDistance = 10000;
        int count = 0;
        avgX = 0;
        avgY = 0;
        Ekeys = enemies.keySet();
        for (String key : Ekeys) {
            if (key.contains("BorderGuard") == false) {
                ene = enemies.get(key);
                if (nearDistance > ene.distance) {
                    nearDistance = ene.distance;
                    nearTank = key;
                    tag = ene;
                }
                System.out.println("[" + key + "]"
                        + "\nX: " + ene.X
                        + "\nY: " + ene.Y
                        + "\nVelocity: " + ene.velocity
                        + "\nChangeHead: " + ene.changehead
                        + "\nGuessAccuracy: " + ene.guessAccuracy
                        + "\nHitCount: " + ene.hitCount
                        + "\n------------------------");
                avgX += ene.X;
                avgY += ene.Y;
                count++;
            }
        }
        avgX = avgX / count;
        avgY = avgY / count;
        avgDis = Point2D.distance(avgX, avgY, getX(), getY());
        System.out.println("Near: " + nearTank
                + "\nAvgPoint: " + avgX + "," + avgY
                + "\n------------------------");
    }

    public void guessGun() {
        Point2D.Double point;
        double turnRad;
        double bulletSpeed;
        double nextDis;
        bulletSpeed = 20 - 3 * firePower;
        point = guess(tag.distance, bulletSpeed);
        nextDis = Point2D.distance(point.x, point.y, getX(), getY());
        point = guess(nextDis, bulletSpeed);
        turnRad = Math.atan2(point.x - getX(), point.y - getY()) - getGunHeadingRadians();
        setTurnGunRightRadians(normalRelativeAngle(turnRad));
        setFire(firePower);
    }

    public void moveTo(double toDeg) {
        double turn;
        double toX, toY;
        double moveAmount = rnd.nextInt(50) + 20;
        if (tag.distance >= avgDis && getOthers() > 2) {
            toX = avgX - getX();
            toY = avgY - getY();
        } else {
            Point2D.Double point;
            double toHead;
            double turnSpeed;
            toHead = normalRelativeAngleDegrees(toDeg + tag.bearing);
            turnSpeed = 10 - 0.75 * getVelocity();
            point = guessL(toHead, turnSpeed);
            toX = point.x - getX();
            toY = point.y - getY();
        }
        turn = Math.atan2(toX, toY) - getHeadingRadians();
        turn = normalRelativeAngleDegrees(toDeg + Math.toDegrees(turn));
        setMaxVelocity(8);
        setTurnRight(turn);
        setAhead(moveAmount * moveDir);
    }

    public Point2D.Double guess(double distanceTo, double speed) {
        double arrivalTime = Math.abs(distanceTo / speed) * tag.guessAccuracy;
        long time = getTime() + Math.round(arrivalTime);
        Point2D.Double point = tag.guessPos(time);
        point = fixFieldPoint(point);
        return point;
    }

    public Point2D.Double guessL(double distanceTo, double speed) {
        double arrivalTime = Math.abs(distanceTo / speed);
        long time = getTime() + Math.round(arrivalTime);
        Point2D.Double point = tag.lineGuess(time);
        point = fixFieldPoint(point);
        return point;
    }

    public Point2D.Double fixFieldPoint(Point2D.Double p) {
        if (p.x > getBattleFieldWidth() - 18) {
            p.x = getBattleFieldWidth() - 18;
        } else if (p.x < 18) {
            p.x = 18;
        }
        if (p.y > getBattleFieldHeight() - 18) {
            p.y = getBattleFieldHeight() - 18;
        } else if (p.y < 18) {
            p.y = 18;
        }
        return p;
    }

    public void moveOnSentry() {
        double toX = centerX - getX();
        double toY = centerY - getY();
        double toAngleCenter = Math.atan2(toX * moveDir, toY * moveDir);
        toAngleCenter = Math.toDegrees(toAngleCenter);
        toAngleCenter = normalRelativeAngleDegrees(toAngleCenter - getHeading());
        setMaxVelocity(5);
        setTurnRight(toAngleCenter);
        setAhead(getSentryBorderSize() * 1.2 * moveDir);
    }

    public boolean checkSentry() {
        return getX() - SENTRY_TOLERANCE <= getSentryBorderSize()
                || getX() + SENTRY_TOLERANCE >= getBattleFieldWidth() - getSentryBorderSize()
                || getY() - SENTRY_TOLERANCE <= getSentryBorderSize()
                || getY() + SENTRY_TOLERANCE >= getBattleFieldHeight() - getSentryBorderSize();
    }

    public void decideFireP(double distance) {
        firePower = (480 + tag.hitCount * 2) / distance;
        if (firePower > FIREPOWER_MAX) {
            firePower = FIREPOWER_MAX;
        } else if (firePower < FIREPOWER_MIN) {
            firePower = FIREPOWER_MIN;
        }
        if (getOthers() == 1 && getEnergy() < 3) {
            firePower = FIREPOWER_MIN;
        }
    }

    public void checkTagFired() {
        if (tag.fired) {
            changeMoveDir();
            tag.fired = false;
        }
    }

    public void radarOnOne() {
        setTurnRadarRight(getBearingThis(getRadarHeading(), tag) + 22.5 * scanDir);
        if (scanDir == FORWARD) {
            scanDir = BACKWARD;
        } else {
            scanDir = FORWARD;
        }
    }

    public double getBearingThis(double thisHead, EnemyRobot t) {
        double bearing = getHeading() + t.bearing - thisHead;
        return normalRelativeAngleDegrees(bearing);
    }

    public void changeMoveDir() {
        long interval = getTime() - changeTime;
        if (interval >= CHANGE_RATE) {
            if (moveDir == FORWARD) {
                moveDir = BACKWARD;
            } else {
                moveDir = FORWARD;
            }
            changeTime = getTime();
        }
    }

    @Override
    public void onRobotDeath(RobotDeathEvent e) {
        enemies.remove(e.getName());
        setTag();
        setTurnRadarRight(getBearingThis(getRadarHeading(), tag)*1.8);
    }

    @Override
    public void onHitWall(HitWallEvent e) {
        changeMoveDir();
        moveOnSentry();
    }

    @Override
    public void onHitRobot(HitRobotEvent e) {
        changeMoveDir();
        if (e.getName().contains("BorderGuard")) {
            moveOnSentry();
        } else if (enemies.containsKey(e.getName())) {
            tag = enemies.get(e.getName());
            tag.bearing = e.getBearing();
            moveTo(180);
            decideFireP(tag.distance);
            guessGun();
            fire(firePower);
        }
    }

    @Override
    public void onBulletMissed(BulletMissedEvent e) {
        tag.hitCount--;
        if (tag.hitCount == 0 && tag.guessAccuracy > 0) {
            tag.guessAccuracy -= GUESS_DECREASE_AMOUNT;
            tag.hitCount = HIT_COUNT_MAX/2;
        } else if (tag.hitCount == 0 && tag.guessAccuracy == 0) {
            tag.guessAccuracy = GUESS_FULL;
            tag.hitCount = HIT_COUNT_MAX;
        }
    }

    @Override
    public void onBulletHit(BulletHitEvent e) {
        if (tag.name.equals(e.getName())) {
            tag.hitCount += 2;
        }
    }

    @Override
    public void onHitByBullet(HitByBulletEvent e) {
        changeMoveDir();
    }

    @Override
    public void onWin(WinEvent e) {
        setAhead(0);
        while (true) {
            setAllColors(new Color(rnd.nextInt(255), rnd.nextInt(255), rnd.nextInt(255)));
            setTurnRadarLeft(20);
            setTurnGunRight(20);
            setTurnLeft(20);
            execute();
        }
    }
}

class EnemyRobot {

    public String name;
    public double distance, bearing, velocity, heading, energy, preEnergy, X, Y;
    public double changehead, preChangehead; // Radian
    public double guessAccuracy;
    public long time;
    public int hitCount;
    public boolean fired;

    public Point2D.Double guessPos(long when) {
        double gX, gY;
        double timeDif = when - time;
        double changeDif = Math.abs(preChangehead - changehead);
        if (Math.abs(changehead) > 0.0001 && changeDif < 0.001) {
            double radius = Math.abs(velocity / changehead);
            double tohead = timeDif * changehead;
            gX = X + (Math.cos(heading) * radius)
                    - (Math.cos(heading + tohead) * radius);
            gY = Y + (Math.sin(heading + tohead) * radius)
                    - (Math.sin(heading) * radius);
        } else {
            gX = X + Math.sin(heading) * velocity * timeDif;
            gY = Y + Math.cos(heading) * velocity * timeDif;
        }
        return new Point2D.Double(gX, gY);
    }

    public Point2D.Double lineGuess(long when) {
        double dif = when - time;
        double gX = X + Math.sin(heading) * velocity * dif;
        double gY = Y + Math.cos(heading) * velocity * dif;
        return new Point2D.Double(gX, gY);
    }
}
