package tileworld.agent;

public class ArdaMessage extends Message {
	public enum MessageType {
		GENERIC,
		INFO,
		INTENTION
	}

	private final MessageType type;
	private final String entityType;
	private final int x;
	private final int y;
	private final int senderX;
	private final int senderY;

	public ArdaMessage(String from, String to, String message, MessageType type, String entityType, int x, int y, int senderX, int senderY) {
		super(from, to, message);
		this.type = type;
		this.entityType = entityType;
		this.x = x;
		this.y = y;
		this.senderX = senderX;
		this.senderY = senderY;
	}

	public static ArdaMessage info(String from, String entityType, int x, int y, int senderX, int senderY) {
		return new ArdaMessage(from, "*", "info", MessageType.INFO, entityType, x, y, senderX, senderY);
	}

	public static ArdaMessage intention(String from, String entityType, int x, int y, int senderX, int senderY) {
		return new ArdaMessage(from, "*", "intent", MessageType.INTENTION, entityType, x, y, senderX, senderY);
	}

	public MessageType getType() {
		return type;
	}

	public String getEntityType() {
		return entityType;
	}

	public int getX() {
		return x;
	}

	public int getY() {
		return y;
	}

	public int getSenderX() {
		return senderX;
	}

	public int getSenderY() {
		return senderY;
	}
}
